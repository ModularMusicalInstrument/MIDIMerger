	;list		p=18f4320	; list directive to define processor


;eeprom stuff available at reset
 

;	global suppress_leading_copy

	global midi_min_volume_copy
	global midi_assign_copy
	global midi_filter_copy

eedata	code_pack  0xf00000

;unsigned char midi_min_volume[] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}; // minimum volume for each midi channel
midi_min_volume_copy
	de 0
	de 0
	de 0x40
	de 0
	de 0
	de 0
	de 0
	de 0
	de 0
	de 0
	de 0
	de 0
	de 0
	de 0
	de 0
	de 0

;unsigned char midi_assign[] = {2,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
midi_assign_copy
	de 2  ; midi 0 is assigned to channel 2
	de 1
	de 2
	de 3
	de 4
	de 5
	de 6
	de 7
	de 8
	de 9
	de 10
	de 11
	de 12
	de 13
	de 14
	de 15

; MIDI filter switch		   0x0a 0x0b 0x0c 0x0d 0x0e 0x0f
;unsigned char midi_filter[] = {1,   1,   1,   1,   1,   1}; // default no filters
midi_filter_copy
	de 1
	de 1
	de 1
	de 1
	de 1
	de 1

the_end
	END                       ; directive 'end of program'
