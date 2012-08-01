all: checksum_test botnet_test

checksum_test: checksum_test.c
	g++ -I ../include -o checksum_test checksum_test.c ../util/checksum.c

botnet_test: botnet_test.c  
	g++ -I ../include -o botnet_test botnet_test.c ../botnet/botnet.c ../util/checksum.c ../util/ascii_print.c
clean:
	rm *.o botnet_test checksum_test
