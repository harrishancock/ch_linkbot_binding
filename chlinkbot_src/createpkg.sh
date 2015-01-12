#!/bin/sh
PACKAGE=chbarobo
VERSION=0.2
PKGDIR=$PACKAGE-$VERSION/$PACKAGE

echo Building $PACKAGE-$VERSION.zip ...
rm -rf $PACKAGE-$VERSION
rm -rf $PACKAGE-$VERSION.zip
mkdir -p $PKGDIR
mkdir $PKGDIR/lib
mkdir $PKGDIR/demos
mkdir $PKGDIR/dl
mkdir $PKGDIR/include
mkdir $PKGDIR/docs
cp chlinkboti.chf $PKGDIR/lib
cp chlinkbotl.chf $PKGDIR/lib
cp liblinkbot.dl $PKGDIR/dl
cp linkbot.h $PKGDIR/include
zip -rq $PACKAGE-$VERSION.zip $PACKAGE-$VERSION
