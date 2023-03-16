#!/bin/bash
for F in {50..1000..50}
do
    N=$((5000))
    python3 rosLatency_senderStaticF_latencyAccumulator.py $F $N
done