#!/bin/bash

CURDIR=`dirname $(readlink -f $0)`;

cd ${CURDIR}/../VTM/

git diff > ${CURDIR}/adaptions_for_vtm_13_0.patch
