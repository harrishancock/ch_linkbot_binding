#!/bin/sh
PACKAGE=chbarobo
VERSION=1.0.6
ARCH=windows-multi
PKGDIR=$PACKAGE-$VERSION-$ARCH/$PACKAGE

echo Building $PACKAGE-$VERSION-$ARCH.zip ...
rm -rf $PACKAGE-$VERSION-$ARCH
rm -rf $PACKAGE-$VERSION-$ARCH.zip
mkdir -p $PKGDIR
mkdir $PKGDIR/lib
mkdir $PKGDIR/demos
mkdir -p $PKGDIR/dl/win32
mkdir -p $PKGDIR/dl/win64
mkdir $PKGDIR/include
mkdir $PKGDIR/docs
mkdir -p $PKGDIR/bin/win32
mkdir -p $PKGDIR/bin/win64
cp chlinkboti.chf $PKGDIR/lib
cp chlinkbotl.chf $PKGDIR/lib
cp build-win32/release/liblinkbot.dl $PKGDIR/dl/win32
cp build-msvc64/Release/liblinkbot.dl $PKGDIR/dl/win64
cp linkbot.h $PKGDIR/include
cp dlls/win32/* $PKGDIR/bin/win32
cp dlls/win64/* $PKGDIR/bin/win64
cp -R dlls/Microsoft.VC80.CRT $PKGDIR/dl
zip -rq $PACKAGE-$VERSION-$ARCH.zip $PACKAGE-$VERSION-$ARCH
