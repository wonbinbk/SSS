##############################################################
# Edit these 2 lines accordingly.
CHIP	= 16f873a 	#used for xc8 compiler command
#PROC	= pic16f873a	#used for pk2cmd command
# End of editable lines.
##############################################################

all:	
	rm -f main.hex
	/opt/microchip/xc8/v1.35/bin/xc8 --chip=$(CHIP) main.c
f:
	pk2cmd -P pic$(CHIP) -R -L8 -J -W -M -Z -F main.hex
r:  
	pk2cmd -P pic$(CHIP) -I && sleep 1s &&pk2cmd -P pic$(CHIP) -R
e:  
	pk2cmd -P pic$(CHIP) -E
