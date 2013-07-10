all: checksum_test pid_test motion_test

test: all run_test

GTEST_LIBS=-lgtest_main -lgtest -lpthread

checksum_test: checksum_test.c
	g++ -I ../util -o bin/checksum_test checksum_test.c ../util/checksum.c $(GTEST_LIBS) 

pid_test: pid_test.c
	g++ -I ../util -o bin/pid_test pid_test.c ../util/pid.c $(GTEST_LIBS) 

motion_test: motion_test.c
	g++ -I ../base_motion -o bin/motion_test motion_test.c ../base_motion/motion_control.c $(GTEST_LIBS) 

run_test:
	./bin/checksum_test
	./bin/pid_test
	./bin/motion_test

clean:
	rm *.o botnet_test checksum_test
