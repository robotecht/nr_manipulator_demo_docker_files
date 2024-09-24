git submodule update --init --recursive

clear &&\
    docker_build.sh &&\
    docker_run.sh \
        "\
            bash \
        "\
        "\
            -v ${PWD}/src:/src \
            --rm \
            --privileged \
            --name demo_pick_and_place \
            --net host \
        "\
        -x  -n
