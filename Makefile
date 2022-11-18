PROJECT = raytrace

# -lm is needed by GCC
LDFLAGS += -lm

RELEASE_FLAGS = -Ofast -Wall -DNDEBUG -fvisibility=hidden -fno-stack-protector \
-fomit-frame-pointer -flto

DEBUG_FLAGS = -O0 -g3 -fno-omit-frame-pointer -Wno-format-security -fno-common \
-fno-optimize-sibling-calls -Wall

ASAN_FLAGS = $(DEBUG_FLAGS)
MSAN_FLAGS = $(DEBUG_FLAGS)
TSAN_FLAGS = $(DEBUG_FLAGS)

# Enable sanitizers if using Clang (but not Apple Clang)
ifeq ($(shell $(CC) -v 2>&1 | grep -v "Apple clang version" | grep -c "clang version"), 1)
    ifneq ($(OS), Windows_NT)
        UNAME_S := $(shell uname -s)
        
        # Enable multithreading support with OpenMP
        # Do not enable when turning in to BBlearn
        # CC_FLAGS += -D OPENMP -Xpreprocessor -fopenmp
        # LDFLAGS += -lomp

        # Enable quadrics
        # CC_FLAGS += -D QUADRICS

        # Enable refraction
        # CC_FLAGS += -D REFRACTION
        
		ASAN_FLAGS += -fsanitize=address,undefined
		TSAN_FLAGS += -fsanitize=thread
        
        ifeq ($(UNAME_S), Linux)
			MSAN_FLAGS += -fsanitize=memory -fsanitize-memory-track-origins=2
        endif
    endif
endif

.PHONY: all debug asan msan tsan clean

all: CC_FLAGS += $(RELEASE_FLAGS)
all: $(PROJECT)
	strip $(PROJECT)

debug: CC_FLAGS += $(DEBUG_FLAGS)
debug: $(PROJECT)

asan: CC_FLAGS += $(ASAN_FLAGS)
asan: $(PROJECT)

msan: CC_FLAGS += $(MSAN_FLAGS)
msan: $(PROJECT)

tsan: CC_FLAGS += $(TSAN_FLAGS)
tsan: $(PROJECT)

clean:
	rm -rf $(PROJECT) *.dSYM *.o

$(PROJECT): $(PROJECT).c ppmrw.c utils.c
	$(CC) $(CC_FLAGS) $^ $(LDFLAGS) -o $@
