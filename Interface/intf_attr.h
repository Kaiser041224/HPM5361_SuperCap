#ifndef INTF_ATTR_H
#define INTF_ATTR_H

/*
 * Toolchain placement attributes shared by App/Interface code.
 * Keep these definitions independent from HPM SDK headers so App code does not
 * need to include hpm_common.h just to place hot paths or fast BSS objects.
 */
#define INTF_ATTR_PLACE_AT(section_name) __attribute__((section(section_name)))

#define INTF_RAMFUNC       INTF_ATTR_PLACE_AT(".fast")
#define INTF_FAST_RAM_BSS  INTF_ATTR_PLACE_AT(".fast_ram.bss")
#define INTF_FAST_RAM_INIT INTF_ATTR_PLACE_AT(".fast_ram.init")

#ifndef ATTR_RAMFUNC
#define ATTR_RAMFUNC INTF_RAMFUNC
#endif

#ifndef ATTR_PLACE_AT_FAST_RAM_BSS
#define ATTR_PLACE_AT_FAST_RAM_BSS INTF_FAST_RAM_BSS
#endif

#ifndef ATTR_PLACE_AT_FAST_RAM_INIT
#define ATTR_PLACE_AT_FAST_RAM_INIT INTF_FAST_RAM_INIT
#endif

#endif /* INTF_ATTR_H */
