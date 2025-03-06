```
docker run --name=mavros_ctrl --hostname=blueos --volume=/dev:/dev:rw --network=host --privileged --log-opt max-size=20m --log-opt max-file=3 --runtime=runc patrickelectric/blueos-ros:0.0.2
```
```
docker exec -it mavros_ctrl /bin/bash
``dd`