.PHONY : kicker

kicker:
	cd kicker && \
	mkdir -p build && cd build && \
	cmake .. && make -j$(nproc) kicker && cd ../.. && \
	cp kicker/build/bin/kicker.nib control/drivers/kicker-programmer/bin/kicker.bin
	
clean:
	rm -rf kicker/build
