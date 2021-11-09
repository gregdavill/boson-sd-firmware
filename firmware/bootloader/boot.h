#ifndef __BOOT_H
#define __BOOT_H

void __attribute__((noreturn)) boot(unsigned long r1, unsigned long r2, unsigned long r3, unsigned long addr);
int serialboot(void);


#endif /* __BOOT_H */
