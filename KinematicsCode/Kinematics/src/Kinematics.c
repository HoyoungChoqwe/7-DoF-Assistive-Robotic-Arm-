#include <math.h>
#include <stdio.h>
#include <string.h>

#include "Kinematics.h"
#include "Servo.h"

#define IK_MAX_ITERS 1200
#define IK_TOL_M 0.001f
#define IK_STEP_SCALE 0.65f
#define IK_DAMPING 0.05f
#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD 0.0174532925f

typedef struct {
    float m[3][3];
} IkMat3;

typedef struct {
    int home_deg;
    int min_deg;
    int max_deg;
} IkJointLimit;

/*
 * Joint order:
 *   0 base, 1 arm pair, 2 lower, 3 middle, 4 top, 5 wrist.
 *
 * World axes:
 *   +X = backward, -X = forward, +Y = left, -Y = right, +Z = up.
 *
 * Lengths are meters. This file is intentionally position-only again:
 * first make FK match real measurements, then trust IK.
 */
static const float BASE_HEIGHT_M = 0.12286f;

static const IkVec3 JOINT_AXIS_LOCAL[IK_DOF] = {
    {0.0f, 0.0f, 1.0f},  /* base yaw */
    {0.0f, 1.0f, 0.0f},  /* arm pitch */
    {0.0f, 1.0f, 0.0f},  /* lower pitch */
    {0.0f, 0.0f, 1.0f},  /* middle yaw */
    {0.0f, 1.0f, 0.0f},  /* top pitch */
    {1.0f, 0.0f, 0.0f},  /* wrist roll; no position effect with zero link */
};

static const float JOINT_SIGN[IK_DOF] = {
    1.0f,   /* base */
    1.0f,   /* arm: lower degree moves forward */
    1.0f,   /* lower: lower degree moves forward */
    -1.0f,  /* middle */
    1.0f,   /* top: lower degree moves forward */
    1.0f,   /* wrist */
};

static const float JOINT_SCALE[IK_DOF] = {
    1.0f,   /* base */
    1.0f,   /* arm: A50..A150 is about 180 physical degrees */
    1.0f,   /* lower */
    1.0f,   /* middle */
    1.0f,   /* top */
    1.0f,   /* wrist */
};

static const IkVec3 LINK_OFFSET_LOCAL[IK_DOF] = {
    {0.02500f, 0.0f, 0.11900f},  /* base axis to arm pair axis: 25 mm out, 119 mm up */
    {0.0f, 0.0f, 0.05950f},      /* arm pair axis to lower axis: 59.5 mm */
    {0.0f, 0.0f, 0.07800f},      /* lower axis to middle axis: 78 mm */
    {0.0f, 0.0f, 0.07100f},      /* middle axis to top axis: 71 mm */
    {0.0f, 0.0f, 0.12555f},      /* top axis to gripper point: 125.55 mm */
    {0.0f, 0.0f, 0.0f},          /* wrist and gripper same modeled point */
};

static const IkJointLimit JOINT_LIMITS[IK_DOF] = {
    {90, 0, 180},   /* base */
    {90, 30, 150},  /* arm */
    {124, 65, 270}, /* lower */
    {158, 81, 248}, /* middle */
    {74, 0, 160},   /* top */
    {90, 0, 180},   /* wrist */
};

static IkVec3 v3(float x, float y, float z)
{
    IkVec3 v = {x, y, z};
    return v;
}

static IkVec3 vadd(IkVec3 a, IkVec3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static IkVec3 vsub(IkVec3 a, IkVec3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static IkVec3 vscale(IkVec3 v, float s) { return v3(v.x * s, v.y * s, v.z * s); }
static float vdot(IkVec3 a, IkVec3 b) { return (a.x * b.x) + (a.y * b.y) + (a.z * b.z); }

static IkVec3 vcross(IkVec3 a, IkVec3 b)
{
    return v3((a.y * b.z) - (a.z * b.y),
              (a.z * b.x) - (a.x * b.z),
              (a.x * b.y) - (a.y * b.x));
}

static float vnorm(IkVec3 v)
{
    return sqrtf(vdot(v, v));
}

static IkVec3 vunit(IkVec3 v)
{
    float n = vnorm(v);
    return (n > 1e-12f) ? vscale(v, 1.0f / n) : v3(0.0f, 0.0f, 0.0f);
}

static IkMat3 mat3_identity(void)
{
    IkMat3 R = {{{1.0f, 0.0f, 0.0f},
                 {0.0f, 1.0f, 0.0f},
                 {0.0f, 0.0f, 1.0f}}};
    return R;
}

static IkMat3 mat3_mul(IkMat3 A, IkMat3 B)
{
    IkMat3 R = {{{0}}};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                R.m[i][j] += A.m[i][k] * B.m[k][j];
            }
        }
    }
    return R;
}

static IkVec3 mat3_mul_vec(IkMat3 R, IkVec3 v)
{
    return v3((R.m[0][0] * v.x) + (R.m[0][1] * v.y) + (R.m[0][2] * v.z),
              (R.m[1][0] * v.x) + (R.m[1][1] * v.y) + (R.m[1][2] * v.z),
              (R.m[2][0] * v.x) + (R.m[2][1] * v.y) + (R.m[2][2] * v.z));
}

static IkMat3 rot_axis_angle(IkVec3 axis_unit, float angle)
{
    float x = axis_unit.x;
    float y = axis_unit.y;
    float z = axis_unit.z;
    float c = cosf(angle);
    float s = sinf(angle);
    float C = 1.0f - c;

    IkMat3 R = {{{c + (x * x * C), (x * y * C) - (z * s), (x * z * C) + (y * s)},
                 {(y * x * C) + (z * s), c + (y * y * C), (y * z * C) - (x * s)},
                 {(z * x * C) - (y * s), (z * y * C) + (x * s), c + (z * z * C)}}};
    return R;
}

static IkVec3 link_vec_local(int i)
{
    return LINK_OFFSET_LOCAL[i];
}

static float servo_deg_to_q(int i, int servo_deg)
{
    return JOINT_SIGN[i] * JOINT_SCALE[i] *
           (servo_deg - JOINT_LIMITS[i].home_deg) * DEG_TO_RAD;
}

static void clamp_q(float q[IK_DOF])
{
    for (int i = 0; i < IK_DOF; i++) {
        float q_a = servo_deg_to_q(i, JOINT_LIMITS[i].min_deg);
        float q_b = servo_deg_to_q(i, JOINT_LIMITS[i].max_deg);
        float min_q = (q_a < q_b) ? q_a : q_b;
        float max_q = (q_a > q_b) ? q_a : q_b;

        if (q[i] < min_q) {
            q[i] = min_q;
        }
        if (q[i] > max_q) {
            q[i] = max_q;
        }
    }
}

static void fk_position(const float q[IK_DOF],
                        IkVec3 p_joint[IK_DOF],
                        IkVec3 w_axis[IK_DOF],
                        IkVec3 *p_ee)
{
    IkMat3 R = mat3_identity();
    IkVec3 p = v3(0.0f, 0.0f, BASE_HEIGHT_M);

    for (int i = 0; i < IK_DOF; i++) {
        p_joint[i] = p;

        IkVec3 a_local_u = vunit(JOINT_AXIS_LOCAL[i]);
        w_axis[i] = vunit(mat3_mul_vec(R, a_local_u));

        R = mat3_mul(R, rot_axis_angle(a_local_u, q[i]));
        p = vadd(p, mat3_mul_vec(R, link_vec_local(i)));
    }

    *p_ee = p;
}

static void jacobian_pos(const float q[IK_DOF], float J[3][IK_DOF], IkVec3 *p_ee_out)
{
    IkVec3 pj[IK_DOF];
    IkVec3 wa[IK_DOF];
    IkVec3 pee;

    fk_position(q, pj, wa, &pee);
    if (p_ee_out != NULL) {
        *p_ee_out = pee;
    }

    for (int i = 0; i < IK_DOF; i++) {
        IkVec3 r = vsub(pee, pj[i]);
        IkVec3 c = vcross(wa[i], r);
        J[0][i] = c.x;
        J[1][i] = c.y;
        J[2][i] = c.z;
    }
}

static int inv3(const float A[3][3], float invA[3][3])
{
    float det = (A[0][0] * ((A[1][1] * A[2][2]) - (A[1][2] * A[2][1]))) -
                (A[0][1] * ((A[1][0] * A[2][2]) - (A[1][2] * A[2][0]))) +
                (A[0][2] * ((A[1][0] * A[2][1]) - (A[1][1] * A[2][0])));

    if (fabsf(det) < 1e-10f) {
        return 0;
    }

    float id = 1.0f / det;
    invA[0][0] = ((A[1][1] * A[2][2]) - (A[1][2] * A[2][1])) * id;
    invA[0][1] = ((A[0][2] * A[2][1]) - (A[0][1] * A[2][2])) * id;
    invA[0][2] = ((A[0][1] * A[1][2]) - (A[0][2] * A[1][1])) * id;
    invA[1][0] = ((A[1][2] * A[2][0]) - (A[1][0] * A[2][2])) * id;
    invA[1][1] = ((A[0][0] * A[2][2]) - (A[0][2] * A[2][0])) * id;
    invA[1][2] = ((A[0][2] * A[1][0]) - (A[0][0] * A[1][2])) * id;
    invA[2][0] = ((A[1][0] * A[2][1]) - (A[1][1] * A[2][0])) * id;
    invA[2][1] = ((A[0][1] * A[2][0]) - (A[0][0] * A[2][1])) * id;
    invA[2][2] = ((A[0][0] * A[1][1]) - (A[0][1] * A[1][0])) * id;
    return 1;
}

static int ik_solve_xyz(IkVec3 target, float q[IK_DOF], IkVec3 *actual)
{
    float lambda = IK_DAMPING;

    clamp_q(q);

    for (int iter = 0; iter < IK_MAX_ITERS; iter++) {
        float J[3][IK_DOF];
        IkVec3 pee;

        jacobian_pos(q, J, &pee);
        IkVec3 e = vsub(target, pee);
        if (vnorm(e) < IK_TOL_M) {
            if (actual != NULL) {
                *actual = pee;
            }
            return 1;
        }

        float JJt[3][3] = {{0}};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < IK_DOF; k++) {
                    JJt[i][j] += J[i][k] * J[j][k];
                }
            }
        }

        JJt[0][0] += lambda * lambda;
        JJt[1][1] += lambda * lambda;
        JJt[2][2] += lambda * lambda;

        float invJJt[3][3];
        if (!inv3(JJt, invJJt)) {
            lambda *= 2.0f;
            continue;
        }

        float ev[3] = {e.x, e.y, e.z};
        float tmpv[3] = {0.0f, 0.0f, 0.0f};
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                tmpv[j] += invJJt[j][k] * ev[k];
            }
        }

        for (int i = 0; i < IK_DOF; i++) {
            float dq = 0.0f;
            for (int j = 0; j < 3; j++) {
                dq += J[j][i] * tmpv[j];
            }

            q[i] += IK_STEP_SCALE * dq;
            if (!isfinite(q[i])) {
                return 0;
            }
        }
        clamp_q(q);
    }

    if (actual != NULL) {
        IkVec3 pj[IK_DOF];
        IkVec3 wa[IK_DOF];
        fk_position(q, pj, wa, actual);
    }
    return 0;
}

static int q_to_servo_deg(int i, float q)
{
    int deg;
    float servo_offset = (JOINT_SIGN[i] * q * RAD_TO_DEG) / JOINT_SCALE[i];

    if (servo_offset >= 0.0f) {
        deg = JOINT_LIMITS[i].home_deg + (int)(servo_offset + 0.5f);
    } else {
        deg = JOINT_LIMITS[i].home_deg + (int)(servo_offset - 0.5f);
    }

    if (deg < JOINT_LIMITS[i].min_deg) {
        deg = JOINT_LIMITS[i].min_deg;
    }
    if (deg > JOINT_LIMITS[i].max_deg) {
        deg = JOINT_LIMITS[i].max_deg;
    }
    return deg;
}

int Kinematics_SolveXYZ(IkVec3 target, IkServoAngles *angles, IkVec3 *actual)
{
    if (angles == NULL) {
        return 0;
    }

    float q[IK_DOF] = {0.0f};
    IkVec3 actual_model;
    int ok = ik_solve_xyz(target, q, &actual_model);

    if (actual != NULL) {
        *actual = actual_model;
    }

    angles->base = q_to_servo_deg(0, q[0]);
    angles->arm = q_to_servo_deg(1, q[1]);
    angles->lower = q_to_servo_deg(2, q[2]);
    angles->middle = q_to_servo_deg(3, q[3]);
    angles->top = q_to_servo_deg(4, q[4]);
    angles->wrist = q_to_servo_deg(5, q[5]);

    return ok;
}

void Kinematics_CurrentXYZ(IkVec3 *actual)
{
    if (actual == NULL) {
        return;
    }

    float q[IK_DOF] = {
        servo_deg_to_q(0, Get_Base()),
        servo_deg_to_q(1, Get_Arm()),
        servo_deg_to_q(2, Get_Lower()),
        servo_deg_to_q(3, Get_Middle()),
        servo_deg_to_q(4, Get_Top()),
        servo_deg_to_q(5, Get_Wrist()),
    };

    IkVec3 pj[IK_DOF];
    IkVec3 wa[IK_DOF];
    fk_position(q, pj, wa, actual);
}

void Kinematics_FKAngles(const IkServoAngles *angles, IkVec3 *actual)
{
    if (angles == NULL || actual == NULL) {
        return;
    }

    float q[IK_DOF] = {
        servo_deg_to_q(0, angles->base),
        servo_deg_to_q(1, angles->arm),
        servo_deg_to_q(2, angles->lower),
        servo_deg_to_q(3, angles->middle),
        servo_deg_to_q(4, angles->top),
        servo_deg_to_q(5, angles->wrist),
    };

    IkVec3 pj[IK_DOF];
    IkVec3 wa[IK_DOF];
    fk_position(q, pj, wa, actual);
}

void Kinematics_ApplyAngles(const IkServoAngles *angles)
{
    if (angles == NULL) {
        return;
    }

    Servo_SetArmPose(angles->base,
                     angles->arm,
                     angles->wrist,
                     angles->top,
                     angles->middle,
                     angles->lower);
}
