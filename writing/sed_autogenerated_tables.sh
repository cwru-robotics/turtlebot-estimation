#!/bin/bash

find autogenerated_tables/ -type f -exec sed -r -i.bak 's/\& ([0-9]+\.[0-9]+)/\& \\num{\1}/g' {} \;