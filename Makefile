SRC_DIR=$(shell pwd)
BUILD_DIR=/tmp/$(shell basename ${SRC_DIR})_build

define COVERAGERC
[run]
include = *src/capabilities*

endef
export COVERAGERC

coverage:
	@echo "Using SRC_DIR: ${SRC_DIR}"
	@echo "Using BUILD_DIR: ${BUILD_DIR}"
	mkdir -p ${BUILD_DIR}
	echo "$$COVERAGERC" > ${BUILD_DIR}/.coveragerc
	@echo "Cleaning out old coverage files"
	-rm ~/.ros/.coverage
	-rm ${BUILD_DIR}/.coverage
	-rm ./.coverage
	-cd ${BUILD_DIR} && mkdir -p src
	-ln -s ${SRC_DIR} ${BUILD_DIR}/src
	cd ${BUILD_DIR} && catkin_make
	cd ${BUILD_DIR} && catkin_make tests
	cd ${BUILD_DIR} && catkin_make -j1 run_tests
	catkin_test_results ${BUILD_DIR}
	ls ${HOME}/.ros/.coverage
	cp ${HOME}/.ros/.coverage ./.coverage.1
	cd ${BUILD_DIR} && ${BUILD_DIR}/devel/env.sh nosetests --where=${SRC_DIR}/test/unit --with-coverage -s
	ls ${BUILD_DIR}/.coverage
	cp ${BUILD_DIR}/.coverage ./.coverage.2
	coverage combine
	coverage report --include='*capabilities/src*' -m
