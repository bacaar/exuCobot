#!/bin/bash
for F in {50..1000..50}
do
    #N=$(( 5*F ))
    N=$((5000))
    python3 rosLatency_senderStaticF.py $F $N
done