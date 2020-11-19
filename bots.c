#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>

#define TIME_STEP 256
#define num_prox_sensor 8
#define QtddLeds 10

WbDeviceTag Leds[QtddLeds];

int main(int argc, char **argv)
{

    int i = 0;
    int times = 0;
    char text[256];
    bool moveBox = false;
    int moveBoxes = 0;
    double proxSensor[num_prox_sensor];
    double rightAcelerator = 1.0, leftAcelerator = 1.0;
    for (i = 0; i < 256; i++)
        text[i] = '0';

    wb_robot_init();
    WbDeviceTag leftEngine, rightEngine;
    leftEngine = wb_robot_get_device("left wheel motor");
    rightEngine = wb_robot_get_device("right wheel motor");
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("ePuck");
    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
    const double *posicao;

    wb_motor_set_position(leftEngine, INFINITY);
    wb_motor_set_position(rightEngine, INFINITY);
    wb_motor_set_velocity(leftEngine, 0);
    wb_motor_set_velocity(rightEngine, 0);

    WbDeviceTag prox_sensor[num_prox_sensor];

    prox_sensor[0] = wb_robot_get_device("ps0");
    wb_distance_sensor_enable(prox_sensor[0], TIME_STEP);
    Leds[0] = wb_robot_get_device("led0");
    wb_led_set(Leds[0], 0);
    prox_sensor[1] = wb_robot_get_device("ps1");
    wb_distance_sensor_enable(prox_sensor[1], TIME_STEP);
    Leds[1] = wb_robot_get_device("led1");
    wb_led_set(Leds[1], 0);
    prox_sensor[2] = wb_robot_get_device("ps2");
    wb_distance_sensor_enable(prox_sensor[2], TIME_STEP);
    Leds[2] = wb_robot_get_device("led2");
    wb_led_set(Leds[2], 0);
    prox_sensor[3] = wb_robot_get_device("ps3");
    wb_distance_sensor_enable(prox_sensor[3], TIME_STEP);
    Leds[3] = wb_robot_get_device("led3");
    wb_led_set(Leds[3], 0);
    prox_sensor[4] = wb_robot_get_device("ps4");
    wb_distance_sensor_enable(prox_sensor[4], TIME_STEP);
    Leds[4] = wb_robot_get_device("led4");
    wb_led_set(Leds[4], 0);
    prox_sensor[5] = wb_robot_get_device("ps5");
    wb_distance_sensor_enable(prox_sensor[5], TIME_STEP);
    Leds[5] = wb_robot_get_device("led5");
    wb_led_set(Leds[5], 0);
    prox_sensor[6] = wb_robot_get_device("ps6");
    wb_distance_sensor_enable(prox_sensor[6], TIME_STEP);
    Leds[6] = wb_robot_get_device("led6");
    wb_led_set(Leds[6], 0);
    prox_sensor[7] = wb_robot_get_device("ps7");
    wb_distance_sensor_enable(prox_sensor[7], TIME_STEP);
    Leds[7] = wb_robot_get_device("led7");
    wb_led_set(Leds[7], 0);

    while (wb_robot_step(TIME_STEP) != -1)
    {
        posicao = wb_supervisor_field_get_sf_vec3f(trans_field);
        for (i = 0; i < 256; i++)
            text[i] = 0;

        for (i = 0; i < num_prox_sensor; i++)
        {
            proxSensor[i] = wb_distance_sensor_get_value(prox_sensor[i]); //sem 60
            sprintf(text, "%s|%d: %5.2f  ", text, i, proxSensor[i]);
        }

        if (posicao[0] > -0.64 && posicao[0] < -0.32 && posicao[2] > -0.64 && posicao[2] < -0.32)
        {

            printf("if %s\n", text);
            leftAcelerator = 1;
            rightAcelerator = 1;

            if (proxSensor[1] > 1000 || proxSensor[0] > 1000 || proxSensor[6] > 1000 || proxSensor[7] > 1000)
            {
                wb_led_set(Leds[0], 1);
                wb_robot_step(165);
                wb_led_set(Leds[0], 0);
                wb_robot_step(165);

                wb_led_set(Leds[1], 1);
                wb_robot_step(165);
                wb_led_set(Leds[1], 0);
                wb_robot_step(165);

                wb_led_set(Leds[2], 1);
                wb_robot_step(165);
                wb_led_set(Leds[2], 0);
                wb_robot_step(165);
                moveBox = true;
                moveBoxes++;

                printf("Vezes que detectou uma caixa movel: %d;\n", moveBoxes);

                for (i = 0; i < moveBoxes; i++)
                {
                    wb_led_set(Leds[i], 1);
                }
            }
        }
        else if (proxSensor[7] > 390)
        {
            rightAcelerator = -0.17;
            leftAcelerator = 1.1;
        }
        else if (proxSensor[0] > 390)
        {
            rightAcelerator = 1.1;
            leftAcelerator = -0.17;
        }
        else if (proxSensor[1] > 390)
        {
            rightAcelerator = 1;
            leftAcelerator = -0.18;
        }
        else if (proxSensor[6] > 390)
        {
            rightAcelerator = -0.18;
            leftAcelerator = 1;
        }
        else if (proxSensor[5] > 390)
        {
            rightAcelerator = -0.23;
            leftAcelerator = 1.1;
        }
        else if (proxSensor[2] > 390)
        {
            rightAcelerator = 1.1;
            leftAcelerator = -0.23;
        }
        else
        {
            rightAcelerator = 1;
            leftAcelerator = 1;
        }

        wb_motor_set_velocity(leftEngine, 4.85 * leftAcelerator);
        wb_motor_set_velocity(rightEngine, 4.85 * rightAcelerator);

        times++;
    };

    wb_robot_cleanup();

    return 0;
}
