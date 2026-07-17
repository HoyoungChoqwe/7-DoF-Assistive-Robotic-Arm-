/*
 * ECE167 Servo Control - main.c
 * Board: ST Nucleo F446RE
 *
 * Controls servos via USB serial commands at 115200 baud.
 */

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <BOARD.h>
#include <Timers.h>
#include "Kinematics.h"
#include "Servo.h"

#define LINE_LEN 48
#define SWEEP_STEP_DEG 5
#define SWEEP_DELAY_MS 180
#define MOVE_STEP_DELAY_MS 45

static void delay_ms(uint32_t ms)
{
    uint32_t start = Timers_GetMilliSeconds();
    while ((Timers_GetMilliSeconds() - start) < ms) {
    }
}

static void print_status(void)
{
    printf("G:%3d A:%3d B:%3d W:%3d T:%3d M:%3d L:%3d\r\n",
           Get_Gripper(), Get_Arm(), Get_Base(),
           Get_Wrist(), Get_Top(), Get_Middle(),
           Get_Lower());
}

static void print_angles(const IkServoAngles *angles)
{
    if (angles == NULL) {
        return;
    }

    printf("angles B:%3d A:%3d W:%3d T:%3d M:%3d L:%3d\r\n",
           angles->base,
           angles->arm,
           angles->wrist,
           angles->top,
           angles->middle,
           angles->lower);
}

static void print_help(void)
{
    printf("Commands:\r\n");
    printf("  a +5     add 5 degrees to paired arm servos, clamped to 30-150\r\n");
    printf("  a -5     subtract 5 degrees from paired arm servos, clamped to 30-150\r\n");
    printf("  a 90     set paired arm servos to 90\r\n");
    printf("  t +5     add 5 degrees to top joint, clamped to 0-160\r\n");
    printf("  t -5     subtract 5 degrees from top joint, clamped to 0-160\r\n");
    printf("  b +5     add 5 degrees to base, clamped to 0-180\r\n");
    printf("  b -5     subtract 5 degrees from base, clamped to 0-180\r\n");
    printf("  l +5     add 5 degrees to lower joint, clamped to 104-265\r\n");
    printf("  l -5     subtract 5 degrees from lower joint, clamped to 104-265\r\n");
    printf("  g/b/w/t/m/l 90  set gripper/base/wrist/top/middle/lower\r\n");
    printf("  g/b/w/t/m/l sweep  continuously sweep one servo\r\n");
    printf("  xyz .10 .05 .20  move end position target in meters\r\n");
    printf("  ik .10 .05 .20   dry-run IK target, print angles, do not move\r\n");
    printf("  fkang B A W T M L  print model XYZ for typed servo angles\r\n");
    printf("  iktest   run canned IK dry-run targets without moving\r\n");
    printf("  fk       print model XYZ for current servo pose\r\n");
    printf("  tp 500   set raw top PWM pulse width in us/ticks for scope test\r\n");
    printf("  sweep    continuously sweep arm\r\n");
    printf("  base is clamped to 0-180\r\n");
    printf("  gripper is clamped to 100-250\r\n");
    printf("  top is clamped to 0-160, home 74\r\n");
    printf("  arm is clamped to 30-150, home 90\r\n");
    printf("  middle is clamped to 81-248, home 158; lower is clamped to 104-265, home 180\r\n");
    printf("  s        status\r\n");
    printf("  r        reset all servos\r\n");
}

static int set_value(const char *name, int value);

static void print_ik_result(const char *label, int ok, IkVec3 target,
                            const IkServoAngles *angles, IkVec3 actual)
{
    printf("%s %s  target=(%.3f %.3f %.3f) actual=(%.3f %.3f %.3f)\r\n",
           label,
           ok ? "OK" : "BEST",
           target.x, target.y, target.z,
           actual.x, actual.y, actual.z);
    print_angles(angles);
}

static void run_ik_dryrun(IkVec3 target)
{
    IkServoAngles angles;
    IkVec3 actual;

    int ok = Kinematics_SolveXYZ(target, &angles, &actual);
    print_ik_result("IK-DRY", ok, target, &angles, actual);
}

static void run_ik_tests(void)
{
    static const IkVec3 targets[] = {
        {0.200f, 0.000f, 0.450f},
        {0.170f, 0.000f, 0.420f},
        {0.150f, 0.000f, 0.400f},
        {0.200f, 0.050f, 0.420f},
        {0.200f, -0.050f, 0.420f},
    };

    for (unsigned int i = 0; i < sizeof(targets) / sizeof(targets[0]); i++) {
        run_ik_dryrun(targets[i]);
    }
}

static int lerp_deg(int start, int end, int step, int steps)
{
    return start + ((end - start) * step) / steps;
}

static IkServoAngles current_angles(void)
{
    IkServoAngles angles;

    angles.base = Get_Base();
    angles.arm = Get_Arm();
    angles.wrist = Get_Wrist();
    angles.top = Get_Top();
    angles.middle = Get_Middle();
    angles.lower = Get_Lower();

    return angles;
}

static void apply_angles_slow(const IkServoAngles *target)
{
    if (target == NULL) {
        return;
    }

    IkServoAngles start = current_angles();
    int max_delta = abs(target->base - start.base);

    if (abs(target->arm - start.arm) > max_delta) max_delta = abs(target->arm - start.arm);
    if (abs(target->wrist - start.wrist) > max_delta) max_delta = abs(target->wrist - start.wrist);
    if (abs(target->top - start.top) > max_delta) max_delta = abs(target->top - start.top);
    if (abs(target->middle - start.middle) > max_delta) max_delta = abs(target->middle - start.middle);
    if (abs(target->lower - start.lower) > max_delta) max_delta = abs(target->lower - start.lower);

    if (max_delta == 0) {
        Kinematics_ApplyAngles(target);
        return;
    }

    for (int step = 1; step <= max_delta; step++) {
        IkServoAngles next;

        next.base = lerp_deg(start.base, target->base, step, max_delta);
        next.arm = lerp_deg(start.arm, target->arm, step, max_delta);
        next.wrist = lerp_deg(start.wrist, target->wrist, step, max_delta);
        next.top = lerp_deg(start.top, target->top, step, max_delta);
        next.middle = lerp_deg(start.middle, target->middle, step, max_delta);
        next.lower = lerp_deg(start.lower, target->lower, step, max_delta);

        Kinematics_ApplyAngles(&next);
        delay_ms(MOVE_STEP_DELAY_MS);
    }
}

static void reset_all_slow(void)
{
    IkServoAngles home;

    home.base = 90;
    home.arm = 90;
    home.wrist = 90;
    home.top = 74;
    home.middle = 158;
    home.lower = 180;

    apply_angles_slow(&home);
    Set_Gripper(100);
}

static int servo_min(const char *name)
{
    if (strcmp(name, "g") == 0) return 100;
    if (strcmp(name, "a") == 0) return 30;
    if (strcmp(name, "b") == 0) return 0;
    if (strcmp(name, "w") == 0) return 0;
    if (strcmp(name, "t") == 0) return 0;
    if (strcmp(name, "m") == 0) return 81;
    if (strcmp(name, "l") == 0) return 104;
    return 0;
}

static int servo_max(const char *name)
{
    if (strcmp(name, "g") == 0) return 250;
    if (strcmp(name, "a") == 0) return 150;
    if (strcmp(name, "b") == 0) return 180;
    if (strcmp(name, "w") == 0) return 180;
    if (strcmp(name, "t") == 0) return 160;
    if (strcmp(name, "m") == 0) return 248;
    if (strcmp(name, "l") == 0) return 265;
    return 0;
}

static int servo_home(const char *name)
{
    if (strcmp(name, "g") == 0) return 100;
    if (strcmp(name, "a") == 0) return 90;
    if (strcmp(name, "b") == 0) return 90;
    if (strcmp(name, "w") == 0) return 90;
    if (strcmp(name, "t") == 0) return 74;
    if (strcmp(name, "m") == 0) return 158;
    if (strcmp(name, "l") == 0) return 180;
    return 0;
}

static void sweep_servo(const char *name)
{
    int min_value = servo_min(name);
    int max_value = servo_max(name);
    int value = servo_home(name);
    int direction = 1;

    if (!set_value(name, value)) {
        printf("Unknown servo '%s'. Type h for help.\r\n", name);
        return;
    }

    printf("Continuous sweep %s home %d, range %d <-> %d\r\n",
           name, value, min_value, max_value);
    Servo_ResetAll();
    set_value(name, value);
    print_status();
    delay_ms(500);

    while (1) {
        value += direction * SWEEP_STEP_DEG;
        if (value >= max_value) {
            value = max_value;
            direction = -1;
        } else if (value <= min_value) {
            value = min_value;
            direction = 1;
        }

        set_value(name, value);

        print_status();
        delay_ms(SWEEP_DELAY_MS);
    }
}

static void read_line(char *line, size_t len)
{
    size_t index = 0;

    while (index + 1 < len) {
        int ch = getchar();
        if (ch == '\r' || ch == '\n') {
            break;
        }
        line[index++] = (char) ch;
    }
    line[index] = '\0';
}

static int current_value(const char *name)
{
    if (strcmp(name, "g") == 0) return Get_Gripper();
    if (strcmp(name, "a") == 0) return Get_Arm();
    if (strcmp(name, "b") == 0) return Get_Base();
    if (strcmp(name, "w") == 0) return Get_Wrist();
    if (strcmp(name, "t") == 0) return Get_Top();
    if (strcmp(name, "m") == 0) return Get_Middle();
    if (strcmp(name, "l") == 0) return Get_Lower();
    return 0;
}

static int set_value(const char *name, int value)
{
    if (strcmp(name, "g") == 0)      Set_Gripper(value);
    else if (strcmp(name, "a") == 0) Set_Arm(value);
    else if (strcmp(name, "b") == 0) Set_Base(value);
    else if (strcmp(name, "w") == 0) Set_Wrist(value);
    else if (strcmp(name, "t") == 0) Set_Top(value);
    else if (strcmp(name, "m") == 0) Set_Middle(value);
    else if (strcmp(name, "l") == 0) Set_Lower(value);
    else return 0;

    return 1;
}

static int set_value_slow(const char *name, int value)
{
    IkServoAngles target = current_angles();

    if (strcmp(name, "g") == 0) {
        Set_Gripper(value);
        return 1;
    } else if (strcmp(name, "a") == 0) {
        target.arm = value;
    } else if (strcmp(name, "b") == 0) {
        target.base = value;
    } else if (strcmp(name, "w") == 0) {
        target.wrist = value;
    } else if (strcmp(name, "t") == 0) {
        target.top = value;
    } else if (strcmp(name, "m") == 0) {
        target.middle = value;
    } else if (strcmp(name, "l") == 0) {
        target.lower = value;
    } else {
        return 0;
    }

    apply_angles_slow(&target);
    return 1;
}

static void handle_command(char *line)
{
    char name[8] = {0};
    char value_text[16] = {0};
    float x;
    float y;
    float z;

    for (char *p = line; *p; p++) {
        *p = (char)tolower((unsigned char)*p);
    }

    if (sscanf(line, "%7s %15s", name, value_text) < 1) {
        return;
    }

    if (strcmp(name, "s") == 0 || strcmp(name, "status") == 0) {
        print_status();
        return;
    }

    if (strcmp(name, "r") == 0 || strcmp(name, "reset") == 0) {
        reset_all_slow();
        print_status();
        return;
    }

    if (strcmp(name, "sweep") == 0) {
        sweep_servo("a");
        return;
    }

    if (strcmp(name, "h") == 0 || strcmp(name, "help") == 0) {
        print_help();
        return;
    }

    if (strcmp(name, "fk") == 0) {
        IkVec3 actual;
        Kinematics_CurrentXYZ(&actual);
        printf("FK actual=(%.3f %.3f %.3f)\r\n", actual.x, actual.y, actual.z);
        print_status();
        return;
    }

    if (strcmp(name, "iktest") == 0) {
        run_ik_tests();
        return;
    }

    if (strcmp(name, "ik") == 0) {
        IkVec3 target;

        if (sscanf(line, "%7s %f %f %f", name, &x, &y, &z) != 4) {
            printf("Usage: ik X Y Z   (meters, dry-run only)\r\n");
            return;
        }

        target.x = x;
        target.y = y;
        target.z = z;
        run_ik_dryrun(target);
        print_status();
        return;
    }

    if (strcmp(name, "fkang") == 0) {
        IkServoAngles angles;
        IkVec3 actual;

        if (sscanf(line, "%7s %d %d %d %d %d %d",
                   name,
                   &angles.base,
                   &angles.arm,
                   &angles.wrist,
                   &angles.top,
                   &angles.middle,
                   &angles.lower) != 7) {
            printf("Usage: fkang B A W T M L\r\n");
            return;
        }

        Kinematics_FKAngles(&angles, &actual);
        printf("FKANG actual=(%.3f %.3f %.3f)\r\n", actual.x, actual.y, actual.z);
        print_angles(&angles);
        return;
    }

    if (strcmp(name, "xyz") == 0) {
        IkServoAngles angles;
        IkVec3 actual;
        IkVec3 target;

        if (sscanf(line, "%7s %f %f %f", name, &x, &y, &z) != 4) {
            printf("Usage: xyz X Y Z   (meters)\r\n");
            return;
        }

        target.x = x;
        target.y = y;
        target.z = z;

        int ok = Kinematics_SolveXYZ(target, &angles, &actual);
        apply_angles_slow(&angles);

        print_ik_result("IK", ok, target, &angles, actual);
        print_status();
        return;
    }

    if (value_text[0] == '\0') {
        printf("Missing value. Type h for help.\r\n");
        return;
    }

    if (strcmp(value_text, "sweep") == 0) {
        sweep_servo(name);
        return;
    }

    if (strcmp(name, "tp") == 0) {
        TIM3->CCR4 = (uint32_t)atoi(value_text);
        printf("Top raw PWM: TIM3_CCR4=%lu ARR=%lu\r\n",
               (unsigned long)TIM3->CCR4,
               (unsigned long)TIM3->ARR);
        return;
    }

    int value = atoi(value_text);
    if (value_text[0] == '+' || value_text[0] == '-') {
        value += current_value(name);
    }

    if (!set_value_slow(name, value)) {
        printf("Unknown servo '%s'. Type h for help.\r\n", name);
        return;
    }
    print_status();
}

int main(void)
{
    char line[LINE_LEN];

    BOARD_Init();
    Timers_Init();

    printf("ECE129 Servo Control\r\n");

    Servo_Init();
    print_help();
    print_status();

    while (1) {
        printf("> ");
        read_line(line, sizeof(line));
        handle_command(line);
    }
    return 0;
}
