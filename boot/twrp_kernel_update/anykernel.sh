# AnyKernel2 Ramdisk Mod Script
# osm0sis @ xda-developers

## AnyKernel setup
# begin properties
kernel.string=Trident by HASH @ xda-developers
do.initd=1
do.modules=0
do.cleanup=1
# end properties

# shell variables
block=/dev/block/platform/mtk-msdc.0/11120000.msdc0/by-name/boot;
is_slot_device=0;


## AnyKernel methods (DO NOT CHANGE)
# import patching functions/variables - see for reference
. /tmp/anykernel/tools/ak2-core.sh;


## AnyKernel permissions
# set permissions for included ramdisk files
chmod -R 755 $ramdisk


## AnyKernel install
dump_boot;

# begin ramdisk changes

# insert casuals init modding
backup_file init.rc
insert_line init.rc "import /init.apollo.rc" after "import .*\.rc" "import /init.apollo.rc"

# end ramdisk changes

write_boot;

## end install

