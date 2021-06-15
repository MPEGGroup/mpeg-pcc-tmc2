#!/bin/bash

CURDIR=`dirname $(readlink -f $0)`;

cd ${CURDIR}/../HM-16.20+SCM-8.8/

git diff > ${CURDIR}/pcc_me-ext_and_namespace_for_HM-16.20+SCM-8.8.patch
