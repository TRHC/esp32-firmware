#ifndef APP_TASKS_H
#define APP_TASKS_H
    void measure_task(void *);
    void watch_task(void *);
    void display_task(void *);
    void ccs811_ready_task(void *);
    void mqtt_task(void *);
#endif