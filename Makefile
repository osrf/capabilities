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
	cd ${BUILD_DIR} && cmake ${SRC_DIR} -DCMAKE_INSTALL_PREFIX=`pwd`/install
	cd ${BUILD_DIR} && make
	cd ${BUILD_DIR} && make tests
	cd ${BUILD_DIR} && make run_tests
	catkin_test_results ${BUILD_DIR}
	ls ${HOME}/.ros/.coverage
	cp ${HOME}/.ros/.coverage ./.coverage.1
	cd ${BUILD_DIR} && ${BUILD_DIR}/devel/env.sh nosetests --where=${SRC_DIR}/test/unit --with-coverage -s
	ls ${BUILD_DIR}/.coverage
	cp ${BUILD_DIR}/.coverage ./.coverage.2
	coverage combine
	coverage report --include='*capabilities/src*' -m
