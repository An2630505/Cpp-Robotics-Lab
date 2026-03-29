# 编译器设置
CXX = g++
CXXFLAGS = -g -std=c++11 -I/usr/include/eigen3 -I./include

# 自动查找 src 目录下的所有 .cpp 文件
SRCS = $(wildcard src/*.cpp)
# 如果当前目录有 cpp 文件也包含进来
SRCS += $(wildcard *.cpp)

# 目标文件（根据第一个源文件命名）
TARGET = main

# 默认目标
all: $(TARGET)

# 编译规则 - 自动处理所有源文件
$(TARGET): $(SRCS)
	@echo "Found source files: $(SRCS)"
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

# 清理规则
clean:
	rm -f $(TARGET)
	rm -f src/*.o
	rm -f *.o

# 重新编译
rebuild: clean all

# 查看找到的源文件
show-sources:
	@echo "Source files in src/: $(wildcard src/*.cpp)"
	@echo "Source files in root/: $(wildcard *.cpp)"
	@echo "Header files in include/: $(wildcard include/*.h)"

.PHONY: all clean rebuild show-sources
