#!/bin/bash

CURDIR=`dirname $(readlink -f $0)`;

cd ${CURDIR}/../VTM-11.2/

git diff > ${CURDIR}/adaptions_for_vtm_11_2.patch
