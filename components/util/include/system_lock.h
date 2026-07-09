#ifndef SYSTEM_LOCK_H
#define SYSTEM_LOCK_H

#ifdef __cplusplus
extern "C" {
#endif

void init_system_lock(void);
void lock_system_task(void);
void unlock_system_task(void);

#ifdef __cplusplus
}
#endif

#endif // SYSTEM_LOCK_H
