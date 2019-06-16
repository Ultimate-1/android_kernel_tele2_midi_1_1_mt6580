#!/bin/bash
clear
if ! [ -d out ]; then
mkdir out
fi
if ! [ -d out/release ]; then
mkdir out/release
fi
set -e
export KBUILD_BUILD_USER=Ultimate_1
export KBUILD_BUILD_HOST=4pda.ru
export CROSS_COMPILE=/work/android/arm-cortex_a7-linux-gnueabihf-linaro_4.9.3-2015.03/bin/arm-cortex_a7-linux-gnueabihf-
export ARCH=arm
if ! [ -f out/.config ]; then
echo -e "\e[32mКонфигурирование ядра...\e[0m"
make O=out tele2_midi_1_1_defconfig
fi
echo -e "\e[32mСборка ядра...\e[0m"
make O=out -j3
cp ./out/arch/arm/boot/zImage-dtb ./out/release
echo -e "\e[32mУпаковка recovery...\e[0m"
./boot/mkbootimg_tele2 --kernel ./out/arch/arm/boot/zImage-dtb --ramdisk ./boot/initramfs_twrp.cpio.gz --base 0x8dffff00 --cmdline 'bootopt=64S3,32S1,32S1' -o ./out/release/twrp.img
echo -e "\e[32mСборка update.zip...\e[0m"
cp ./out/arch/arm/boot/zImage-dtb ./boot/twrp_kernel_update/zImage
cd ./boot/twrp_kernel_update/
zip -r -0 update.zip *
mv update.zip ../../out/release
rm zImage
echo -e "\e[32mВсе операции завершены!\e[0m"
