#!/bin/bash
rm -f ${1}2.txt
sed  '$ ! {s|$|, |g}' $1.txt >> ${1}2.txt

