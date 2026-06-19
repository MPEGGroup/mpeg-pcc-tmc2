#!/bin/bash

CURDIR=`dirname $(readlink -f $0)`;
HM_VERSION="${1:-HM-16.20+SCM-8.8}"

cd ${CURDIR}/../${HM_VERSION}/

git diff > ${CURDIR}/${HM_VERSION}.patch
