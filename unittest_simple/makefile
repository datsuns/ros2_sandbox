default:
	colcon build

run:

test:
	colcon build
	colcon test --event-handlers console_cohesion+
	colcon test-result --verbose

coverage:
	lcov -d build/simple_unittest/CMakeFiles -c -o all.info
	lcov -r all.info \
		*/googletest/* test/* */c++/* */opt/ros/* rclcpp/** /usr/include/** \
		-o lcov.info
	genhtml -o lcovHtml --num-spaces 4 -s --legend lcov.info

setup:
	sudo apt install lcov

.PHONY: default run
