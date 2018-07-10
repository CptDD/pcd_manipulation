#!/bin/bash

echo "Running aems2"
i=0



while [[ $i -lt $2 ]]; do
	./pomdpOnlineSol --budget 1000 --simLen $1 ../examples/POMDPX/second.pomdpx > ../results/res_$1_$i.txt

	echo "Results ready for :"$i
	let i=i+1
done