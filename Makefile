all: touched

touched: src/eusmodel_template_gen.l
	./src/eusmodel_template_gen.sh
	make -f Makefile.ros
	touch touched

clean:
	rm -rf launch template
	make clean -f Makefile.ros
	rm -f touched