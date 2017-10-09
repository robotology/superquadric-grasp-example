#!/bin/bash

DESTINATIONXML=generated-from-xml
YARP_ROOT=../../../../../../yarp

# clean-up
rm -rf doc 
rm -rf $DESTINATIONXML 

# generate doxy from xml
mkdir $DESTINATIONXML
list=`find .. -iname *.xml | xargs`
for i in $list
do
   filename=`basename $i`
   doxyfile=${filename%%.*}
   xsltproc --output $DESTINATIONXML/$doxyfile.xml.dox $YARP_ROOT/scripts/yarp-module.xsl $i
done

doxygen ./generate.txt
