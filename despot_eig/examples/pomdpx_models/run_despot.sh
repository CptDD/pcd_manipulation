#!/bin/bash

echo "Running despot"
i=0



while [[ $i -lt $1 ]]; do
	./despot_pomdpx -m ./data/gen_ext.pomdpx > results/res_$i.txt


	echo "Results ready for :"$i
	let i=i+1
done