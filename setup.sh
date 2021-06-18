#!/usr/bin/env bash

cd /opt
sudo rm -f "download?cid=B7DCBFD795E0B204&resid=B7DCBFD795E0B204!2793&authkey=AGd1u1wsAuI7I9o"
sudo wget --no-check-certificate "https://onedrive.live.com/download?cid=B7DCBFD795E0B204&resid=B7DCBFD795E0B204%212793&authkey=AGd1u1wsAuI7I9o"

sudo tar xf "download?cid=B7DCBFD795E0B204&resid=B7DCBFD795E0B204!2793&authkey=AGd1u1wsAuI7I9o"

sudo rm -f "download?cid=B7DCBFD795E0B204&resid=B7DCBFD795E0B204!2793&authkey=AGd1u1wsAuI7I9o"

cd

mkdir .environmnetReconstruction