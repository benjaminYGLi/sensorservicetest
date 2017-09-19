/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <inttypes.h>
#include <android/sensor.h>
#include <gui/Sensor.h>
#include <gui/SensorManager.h>
#include <gui/SensorEventQueue.h>
#include <utils/Looper.h>

struct sensorName {
    const char *name;
    int32_t    sensorType;
} sensorName;

/* refers to /hardware/libhardware/include/hardware/sensors.h */
static const struct sensorName sensorNameList[] = {
    { "accelerometer", SENSOR_TYPE_ACCELEROMETER },
    { "magnetic_field", SENSOR_TYPE_GEOMAGNETIC_FIELD },
    { "orientation", SENSOR_TYPE_ORIENTATION },
    { "gyroscope", SENSOR_TYPE_GYROSCOPE },
    { "light", SENSOR_TYPE_LIGHT },
    { "pressure", SENSOR_TYPE_PRESSURE },
    { "temperature", SENSOR_TYPE_TEMPERATURE },
    { "proximity", SENSOR_TYPE_PROXIMITY },
    { "gravity", SENSOR_TYPE_GRAVITY },
    { "linear_acceleration", SENSOR_TYPE_LINEAR_ACCELERATION },
    { "rotation_vector", SENSOR_TYPE_ROTATION_VECTOR },
    { "relative_humidity", SENSOR_TYPE_RELATIVE_HUMIDITY },
    { "ambient_temperature", SENSOR_TYPE_AMBIENT_TEMPERATURE },
    { "magnetic_field_uncalibrated", SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED },
    { "game_rotation_vector", SENSOR_TYPE_GAME_ROTATION_VECTOR },
    { "gyroscope_uncalibrated", SENSOR_TYPE_GYROSCOPE_UNCALIBRATED },
    { "significant_motion", SENSOR_TYPE_SIGNIFICANT_MOTION },
    { "step_detector", SENSOR_TYPE_STEP_DETECTOR },
    { "step_counter", SENSOR_TYPE_STEP_COUNTER },
    { "geomagnetic_rotation_vector", SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR },
    { "heart_rate", SENSOR_TYPE_HEART_RATE },
    { "tilt_detector", SENSOR_TYPE_TILT_DETECTOR },
    { "wake_gesture", SENSOR_TYPE_WAKE_GESTURE },
    { "glance_gesture", SENSOR_TYPE_GLANCE_GESTURE },
    { "pick_up_gesture", SENSOR_TYPE_PICK_UP_GESTURE },
    { "wrist_tilt_gesture", SENSOR_TYPE_WRIST_TILT_GESTURE },
};

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

using namespace android;

static nsecs_t sStartTime = 0;

int receiver(__unused int fd, __unused int events, void* data)
{
    sp<SensorEventQueue> q((SensorEventQueue*)data);
    ssize_t n;
    ASensorEvent buffer[8];

    static nsecs_t oldTimeStamp = 0;
    static nsecs_t lastTime     = 0;

    /* numEvents maybe different for low event rates sensors */
    while ((n = q->read(buffer, 8)) > 0) {
        for (int i=0 ; i<n ; i++) {
            float t;
            if (oldTimeStamp) {
                t = float(buffer[i].timestamp - oldTimeStamp) / ms2ns(1);
            } else {
                t = float(buffer[i].timestamp - sStartTime) / ms2ns(1);
            }
            oldTimeStamp = buffer[i].timestamp;

            float d;
            nsecs_t currentTime = systemTime();
            if (lastTime) {
                d = float(currentTime - lastTime) / ms2ns(1);
            } else {
                d = float(currentTime - sStartTime) / ms2ns(1);
            }
            lastTime = currentTime;

            switch (buffer[i].type) {

            /* refer to /docs/reference/android/hardware/SensorEvent.html#values */
            case SENSOR_TYPE_ACCELEROMETER:
            case SENSOR_TYPE_GEOMAGNETIC_FIELD:
            case SENSOR_TYPE_ORIENTATION:
            case SENSOR_TYPE_GYROSCOPE:
            case SENSOR_TYPE_GRAVITY:
            case SENSOR_TYPE_LINEAR_ACCELERATION:
                printf("%" PRId64 "\t%8f\t%8f\t%8f\t%f\t%f\n",
                        buffer[i].timestamp,
                        buffer[i].data[0], buffer[i].data[1], buffer[i].data[2],
                        t, d);
                break;

            case SENSOR_TYPE_LIGHT:
                printf("%" PRId64 "\t%8f\t%f\n",
                        buffer[i].timestamp, buffer[i].light, t);
                break;

            case SENSOR_TYPE_PRESSURE:
                printf("%" PRId64 "\t%8f\t%f\n",
                        buffer[i].timestamp, buffer[i].pressure, t);
                break;

            case SENSOR_TYPE_TEMPERATURE:
            case SENSOR_TYPE_AMBIENT_TEMPERATURE:
                printf("%" PRId64 "\t%8f\t%f\n",
                        buffer[i].timestamp, buffer[i].temperature, t);
                break;

            case SENSOR_TYPE_PROXIMITY:
                printf("%" PRId64 "\t%8f\t%f\n",
                        buffer[i].timestamp, buffer[i].distance, t);
                break;

            case SENSOR_TYPE_RELATIVE_HUMIDITY:
                printf("%" PRId64 "\t%8f\t%f\n",
                        buffer[i].timestamp, buffer[i].relative_humidity, t);
                break;

            case SENSOR_TYPE_ROTATION_VECTOR:
            case SENSOR_TYPE_GAME_ROTATION_VECTOR:
            case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
                printf("%" PRId64 "\t%8f\t%8f\t%8f\t%8f\t%8f\t%f\n",
                        buffer[i].timestamp,
                        buffer[i].data[0], buffer[i].data[1], buffer[i].data[2],
                        buffer[i].data[3], buffer[i].data[4],
                        t);
                break;

            case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
            case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
                printf("%" PRId64 "\t%8f\t%8f\t%8f\t%8f\t%8f\t%8f\t%f\n",
                        buffer[i].timestamp,
                        buffer[i].data[0], buffer[i].data[1], buffer[i].data[2],
                        buffer[i].data[3], buffer[i].data[4], buffer[i].data[5],
                        t);
                break;

            /* following types of sensor only allowed return value is 1.0 */
            case SENSOR_TYPE_SIGNIFICANT_MOTION:
            case SENSOR_TYPE_STEP_DETECTOR:
            case SENSOR_TYPE_TILT_DETECTOR:
            case SENSOR_TYPE_WAKE_GESTURE:
            case SENSOR_TYPE_GLANCE_GESTURE:
            case SENSOR_TYPE_PICK_UP_GESTURE:
            case SENSOR_TYPE_WRIST_TILT_GESTURE:
                printf("%" PRId64 "\t%8f\t%f\n",
                        buffer[i].timestamp, buffer[i].data[0], t);
                break;

            case SENSOR_TYPE_STEP_COUNTER:
                printf("%" PRId64 "\t%8ld\t%f\n",
                        buffer[i].timestamp, buffer[i].u64.step_counter, t);
                break;

            case SENSOR_TYPE_HEART_RATE:
                printf("%" PRId64 "\t%8f\t%2d\t%f\n",
                        buffer[i].timestamp,
                        buffer[i].heart_rate.bpm, buffer[i].heart_rate.status,
                        t);
                break;

            default:
                break;
            }

        }
    }
    if (n<0 && n != -EAGAIN) {
        printf("error reading events (%s)\n", strerror(-n));
    }
    return 1;
}

#define ACTION_LIST      (1 << 0)
#define ACTION_DELAY     (1 << 1)
#define ACTION_HELP      (1 << 2)
#define ACTION_DUMP      (1 << 3)

#define CMD_LIST         "[--list] [--sensor sensorName [--delay ms]] [--help]"

int main(int argc, char *argv[])
{
    int i, j, action = 0;
    int sensorType = 0xFF;
    int delayMs = 0;

    /* check input parameters */
    if (argc == 1) {
        printf("As following example:\n");
        printf("\tapp %s\n", CMD_LIST);
        return 0;
    }

    if ((argc == 2) && (strcmp(argv[1], "--list") == 0)) {
        action = ACTION_LIST;
    }

    if ((argc == 2) && (strcmp(argv[1], "--help") == 0)) {
        action = ACTION_HELP;
    }

    if (argc == 3 || argc == 5) {
        int k = 1;
        if (strcmp(argv[k], "--sensor") == 0) {
            k++;
            j = ARRAY_SIZE(sensorNameList);
            for (i=0; i<j; i++) {
                if (strcmp(argv[k], sensorNameList[i].name) == 0) {
                     sensorType = sensorNameList[i].sensorType;
                }
            }
            if (sensorType == 0xFF) {
                printf("err: not supported sensor!\n");
                return -1;
            }

            action = ACTION_DUMP;
            k++;
            if ((argc == 5) && (strcmp(argv[k], "--delay") == 0)) {
                k++;
                action |= ACTION_DELAY;
                delayMs = atoi(argv[k]);
            }
        }
    }

    /* handle help cmd firstly */
    if (action == ACTION_HELP) {
        j = ARRAY_SIZE(sensorNameList);
        printf("app %s\n", CMD_LIST);
        printf("supported [%d] sensors, sensorName as following:\n", j);
        for (i=0; i<j; i++) {
            printf("\t%s\n", sensorNameList[i].name);
        }
        return 0;
    }

    SensorManager *mgr;
    mgr = &SensorManager::getInstanceForPackage(String16("Sensor Service Test"));

    Sensor const* const* sensorList;
    ssize_t count = mgr->getSensorList(&sensorList);
    printf("numSensors=%d\n", int(count));

    /* list sensors details */
    if (action == ACTION_LIST) {
        if (count > 0) {
            for (size_t i=0 ; i<size_t(count) ; i++) {
                 Sensor const* const list = sensorList[i];
                 printf("Sensor name: %s\n\tvendor: %s,\ttype: %s\n"
                        "\tminval: %f,\tmaxval: %f\n"
                        "\tresolution: %f,\tpowerusage: %f\n"
                        "\tmindelay: %d,\tmindelayNs: %ld,\tmaxdelay: %d\n"
                        "\tFifoResvEvtCnt: %d,\tFifoMaxEvtCnt: %d\n"
                        "\tversion: %d,\tflag: %d,\thandle: %d\n"
                        "\tisWakeUpSensor: %d,\treportingMode: %d\n"
                        "\tRequiredPermission: %s,\tisRequiredPermissonOnRunTime: %d\n"
                        "\tRequiredAppOp: %d,\ttype: %d\n",
                         list->getName().string(),
                         list->getVendor().string(), list->getStringType().string(),
                         list->getMinValue(), list->getMaxValue(),
                         list->getResolution(), list->getPowerUsage(),
                         list->getMinDelay(), list->getMinDelayNs(), list->getMaxDelay(),
                         list->getFifoReservedEventCount(), list->getFifoMaxEventCount(),
                         list->getVersion(), list->getFlags(), list->getHandle(),
                         list->isWakeUpSensor(), list->getReportingMode(),
                         list->getRequiredPermission().string(),
                         list->isRequiredPermissionRuntime(), list->getRequiredAppOp(),
                         list->getType());
            }
        }
        return 0;
    }

    sp<SensorEventQueue> q = mgr->createEventQueue();
    printf("queue=%p\n", q.get());

    Sensor const* mSensor = mgr->getDefaultSensor(sensorType);
    if (mSensor == NULL) {
        printf("err: there is not any [%s] type sensor inside system!\n",
                sensorNameList[sensorType-1].name);
        return -2;
    }
    printf("mSensor=%p (%s)\n",
            mSensor, mSensor->getName().string());

    sStartTime = systemTime();

    q->enableSensor(mSensor);

    /* set delay if with delay parameter */
    if (action & ACTION_DELAY) {
        printf("delayMs=%d\n", delayMs);
        q->setEventRate(mSensor, ms2ns(delayMs));
    } else {
        q->setEventRate(mSensor, ms2ns(200));
    }

    sp<Looper> loop = new Looper(false);
    loop->addFd(q->getFd(), 0, ALOOPER_EVENT_INPUT, receiver, q.get());

    do {
        //printf("about to poll...\n");
        int32_t ret = loop->pollOnce(-1);
        switch (ret) {
            case ALOOPER_POLL_WAKE:
                //("ALOOPER_POLL_WAKE\n");
                break;
            case ALOOPER_POLL_CALLBACK:
                //("ALOOPER_POLL_CALLBACK\n");
                break;
            case ALOOPER_POLL_TIMEOUT:
                printf("ALOOPER_POLL_TIMEOUT\n");
                break;
            case ALOOPER_POLL_ERROR:
                printf("ALOOPER_POLL_TIMEOUT\n");
                break;
            default:
                printf("ugh? poll returned %d\n", ret);
                break;
        }
    } while (1);


    return 0;
}
