#!/bin/bash
git submodule update --init --recursive

clear &&\
    docker_build.sh &&\
    docker_run.sh \
        "\
            bash \
        "\
        "\
            -v ${PWD}/src:/src \
            -e RUN_MODE=$RUN_MODE \
            --rm \
            --privileged \
            --name demo_pick_and_place \
        "\
        -x  -n
