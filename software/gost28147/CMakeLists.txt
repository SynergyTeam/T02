cmake_minimum_required(VERSION 2.8) # Проверка версии CMake.
									# Если версия установленой программы
									# старее указаной, произойдёт аварийный выход.
project(Gost28147)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

include_directories(.)

list(APPEND sources
		./Gost28147.cpp
		./main.cpp)

add_executable(Gost28147 ${sources})		# Создает исполняемый файл с именем <name>
													# из исходника <name>.cpp