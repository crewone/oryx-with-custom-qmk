#!/bin/bash

cd qmk_firmware
git submodule update --remote
git fetch origin firmware24
git checkout -B firmware24 origin/firmware24
git submodule update --recursive
cd ..

docker build -t qmk .

rm -rf qmk_firmware/keyboards/zsa/voyager/keymaps/YV4Oa
mkdir -p qmk_firmware/keyboards/zsa/voyager/keymaps && cp -r YV4Oa qmk_firmware/keyboards/zsa/voyager/keymaps/

docker run -v ./qmk_firmware:/root --rm qmk /bin/sh -c "
            qmk setup zsa/qmk_firmware -b firmware24 -y
            make zsa/voyager:YV4Oa -j 10
          "

cp qmk_firmware/zsa_voyager_YV4Oa.bin /mnt/d/
