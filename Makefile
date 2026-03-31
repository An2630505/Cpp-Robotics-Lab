# 编译器设置
CXX = g++
CXXFLAGS = -g -std=c++11 -I/usr/include/eigen3 -I./include

# 自动查找 src 目录下的所有 .cpp 文件
SRCS = $(wildcard src/*.cpp)
# 如果当前目录有 cpp 文件也包含进来
SRCS += $(wildcard *.cpp)

# 目标文件（根据第一个源文件命名）
TARGET = ./output/main

# 生成目标文件列表（将 .cpp 替换为 .o）
OBJS = $(patsubst %.cpp, ./build/%.o,$(SRCS))

# 默认目标
all: $(TARGET)

# 链接所有目标文件
$(TARGET): $(OBJS)
	@echo "Linking object files: $(OBJS)"
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(TARGET)

# 模式规则：为每个源文件生成目标文件
# 自动处理 src 目录下的 .cpp 文件
./build/src/%.o: src/%.cpp
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 模式规则：处理根目录下的 .cpp 文件
./build/%.o: %.cpp
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 清理规则
clean:
	rm -f $(TARGET)
	rm -f ./build/src/*.o
	rm -f ./build/*.o

# 重新编译
rebuild: clean all

# 查看找到的源文件
show-sources:
	@echo "Source files in src/: $(wildcard src/*.cpp)"
	@echo "Source files in root/: $(wildcard *.cpp)"
	@echo "Header files in include/: $(wildcard include/*.h)"

.PHONY: all clean rebuild show-sources