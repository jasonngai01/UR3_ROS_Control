@@@ Docker install in ubuntu 16.04
@@@ VincentChan
@@@ 14th-March 2020


$ sudo apt -y update
$ sudo apt -y install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
$ sudo apt remove docker docker-engine docker.io containerd runc
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
$ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
$ sudo apt update
$ sudo apt -y install docker-ce docker-ce-cli containerd.io
$ sudo usermod -aG docker vincent
$ newgrp docker
$ docker version

