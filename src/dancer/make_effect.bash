#!/bin/bash

if [ $# -ne 2 ]; then
    echo " * Syntax  : bash $0 <class_name> <file_name>"
    echo " * Example : bash $0 FooEffect foo_effect"
    exit 1
fi

toUpper() {
    echo $1 | tr "[:lower:]" "[:upper:]"
}

### the name of the target class
CLASS_NAME=$1
### the name of the target filenames
FILE_NAME=$2
### the include guards
INCLUDE_GUARD_NAME=`toUpper $FILE_NAME`

COMPLETE_FILE_NAME=$FILE_NAME.h

cp copy_color_to_out.h $COMPLETE_FILE_NAME
## replace the file names
sed -i "s/copy_color_to_out/${FILE_NAME}/g" $COMPLETE_FILE_NAME
## replace the class names
sed -i "s/CopyColorToOut/${CLASS_NAME}/g" $COMPLETE_FILE_NAME
## replace the include guards
sed -i "s/COPY_COLOR_TO_OUT/${INCLUDE_GUARD_NAME}/g" $COMPLETE_FILE_NAME

### change CMakeLists
sed -i "s,  ### end of effect interfaces,  ${COMPLETE_FILE_NAME}\n  ### end of effect interfaces,g" CMakeLists.txt
### change effect_collection
sed -i "s,// end of effect interfaces includes,#include \"${COMPLETE_FILE_NAME}\"\n// end of effect interfaces includes,g" effect_collection.cpp
sed -i "s,  // end of effect interfaces instantiations,  effects.push_back(new ${CLASS_NAME}());\n  // end of effect interfaces instantiations,g" effect_collection.cpp

echo "* Finished. "
