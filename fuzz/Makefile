BUILD_DIR := build

.PHONY: all clean fuzz afl

all: fuzz

clean:
	rm -rf $(BUILD_DIR)

fuzz: clean
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && CC=clang CXX=clang++ cmake .. && make
	mv $(BUILD_DIR)/harness harness

afl: clean
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && CC=afl-clang-fast CXX=afl-clang-fast++ cmake .. && make
	mv $(BUILD_DIR)/harness harness_afl
