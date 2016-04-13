dir = cpp_code

demo:
	cd $(dir) && $(MAKE) demo

lib:
	cd $(dir) && $(MAKE) lib

clean:
	cd $(dir) && $(MAKE) clean
