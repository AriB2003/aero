PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//18547404/1276551/2.50/28/3/Integrated Circuit

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r67.5_25"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 0.250) (shapeHeight 0.675))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(padStyleDef "r57.5_25"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 0.250) (shapeHeight 0.575))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "BNO085" (originalName "BNO085")
		(multiLayer
			(pad (padNum 1) (padStyleRef r67.5_25) (pt 1.563, 2.250) (rotation 90))
			(pad (padNum 2) (padStyleRef r57.5_25) (pt 0.750, 2.313) (rotation 0))
			(pad (padNum 3) (padStyleRef r57.5_25) (pt 0.250, 2.313) (rotation 0))
			(pad (padNum 4) (padStyleRef r57.5_25) (pt -0.250, 2.313) (rotation 0))
			(pad (padNum 5) (padStyleRef r57.5_25) (pt -0.750, 2.313) (rotation 0))
			(pad (padNum 6) (padStyleRef r67.5_25) (pt -1.563, 2.250) (rotation 90))
			(pad (padNum 7) (padStyleRef r67.5_25) (pt -1.563, 1.750) (rotation 90))
			(pad (padNum 8) (padStyleRef r67.5_25) (pt -1.563, 1.250) (rotation 90))
			(pad (padNum 9) (padStyleRef r67.5_25) (pt -1.563, 0.750) (rotation 90))
			(pad (padNum 10) (padStyleRef r67.5_25) (pt -1.563, 0.250) (rotation 90))
			(pad (padNum 11) (padStyleRef r67.5_25) (pt -1.563, -0.250) (rotation 90))
			(pad (padNum 12) (padStyleRef r67.5_25) (pt -1.563, -0.750) (rotation 90))
			(pad (padNum 13) (padStyleRef r67.5_25) (pt -1.563, -1.250) (rotation 90))
			(pad (padNum 14) (padStyleRef r67.5_25) (pt -1.563, -1.750) (rotation 90))
			(pad (padNum 15) (padStyleRef r67.5_25) (pt -1.563, -2.250) (rotation 90))
			(pad (padNum 16) (padStyleRef r57.5_25) (pt -0.750, -2.313) (rotation 0))
			(pad (padNum 17) (padStyleRef r57.5_25) (pt -0.250, -2.313) (rotation 0))
			(pad (padNum 18) (padStyleRef r57.5_25) (pt 0.250, -2.313) (rotation 0))
			(pad (padNum 19) (padStyleRef r57.5_25) (pt 0.750, -2.313) (rotation 0))
			(pad (padNum 20) (padStyleRef r67.5_25) (pt 1.563, -2.250) (rotation 90))
			(pad (padNum 21) (padStyleRef r67.5_25) (pt 1.563, -1.750) (rotation 90))
			(pad (padNum 22) (padStyleRef r67.5_25) (pt 1.563, -1.250) (rotation 90))
			(pad (padNum 23) (padStyleRef r67.5_25) (pt 1.563, -0.750) (rotation 90))
			(pad (padNum 24) (padStyleRef r67.5_25) (pt 1.563, -0.250) (rotation 90))
			(pad (padNum 25) (padStyleRef r67.5_25) (pt 1.563, 0.250) (rotation 90))
			(pad (padNum 26) (padStyleRef r67.5_25) (pt 1.563, 0.750) (rotation 90))
			(pad (padNum 27) (padStyleRef r67.5_25) (pt 1.563, 1.250) (rotation 90))
			(pad (padNum 28) (padStyleRef r67.5_25) (pt 1.563, 1.750) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0.000, -0.000) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.9 -2.6) (pt 1.9 -2.6) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 1.9 -2.6) (pt 1.9 2.6) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 1.9 2.6) (pt -1.9 2.6) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.9 2.6) (pt -1.9 -2.6) (width 0.025))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.9 3.6) (pt 2.9 3.6) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 2.9 3.6) (pt 2.9 -3.601) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 2.9 -3.601) (pt -2.9 -3.601) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.9 -3.601) (pt -2.9 3.6) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt 2.4 2.25) (pt 2.4 2.25) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(arc (pt 2.45, 2.25) (radius 0.05) (startAngle 180.0) (sweepAngle 180.0) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 2.5 2.25) (pt 2.5 2.25) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(arc (pt 2.45, 2.25) (radius 0.05) (startAngle .0) (sweepAngle 180.0) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 2.4 2.25) (pt 2.4 2.25) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(arc (pt 2.45, 2.25) (radius 0.05) (startAngle 180.0) (sweepAngle 180.0) (width 0.2))
		)
	)
	(symbolDef "BNO085" (originalName "BNO085")

		(pin (pinNum 1) (pt 2000 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1770 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 1100 mils 700 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1125 mils 470 mils) (rotation 90]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 1000 mils 700 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1025 mils 470 mils) (rotation 90]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 4) (pt 900 mils 700 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 925 mils 470 mils) (rotation 90]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 5) (pt 800 mils 700 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 825 mils 470 mils) (rotation 90]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 6) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 7) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 8) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 9) (pt 0 mils -300 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -325 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 10) (pt 0 mils -400 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -425 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 11) (pt 0 mils -500 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -525 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 12) (pt 0 mils -600 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -625 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 13) (pt 0 mils -700 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -725 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 14) (pt 0 mils -800 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -825 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 15) (pt 0 mils -900 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -925 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 16) (pt 800 mils -1900 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 825 mils -1670 mils) (rotation 90]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 17) (pt 900 mils -1900 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 925 mils -1670 mils) (rotation 90]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 18) (pt 1000 mils -1900 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1025 mils -1670 mils) (rotation 90]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 19) (pt 1100 mils -1900 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1125 mils -1670 mils) (rotation 90]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 20) (pt 2000 mils -900 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1770 mils -925 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 21) (pt 2000 mils -800 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1770 mils -825 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 22) (pt 2000 mils -700 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1770 mils -725 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 23) (pt 2000 mils -600 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1770 mils -625 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 24) (pt 2000 mils -500 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1770 mils -525 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 25) (pt 2000 mils -400 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1770 mils -425 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 26) (pt 2000 mils -300 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1770 mils -325 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 27) (pt 2000 mils -200 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1770 mils -225 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 28) (pt 2000 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1770 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 500 mils) (pt 1800 mils 500 mils) (width 6 mils))
		(line (pt 1800 mils 500 mils) (pt 1800 mils -1700 mils) (width 6 mils))
		(line (pt 1800 mils -1700 mils) (pt 200 mils -1700 mils) (width 6 mils))
		(line (pt 200 mils -1700 mils) (pt 200 mils 500 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 1850 mils 700 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 1850 mils 600 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "BNO085" (originalName "BNO085") (compHeader (numPins 28) (numParts 1) (refDesPrefix IC)
		)
		(compPin "1" (pinName "RESV_NC_1") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "GND_1") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "3" (pinName "VDD") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "4" (pinName "BOOTN") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "5" (pinName "PS1") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "6" (pinName "PS0/WAKE") (partNum 1) (symPinNum 6) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "7" (pinName "RESV_NC_2") (partNum 1) (symPinNum 7) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "8" (pinName "RESV_NC_3") (partNum 1) (symPinNum 8) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "9" (pinName "CAP") (partNum 1) (symPinNum 9) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "10" (pinName "CLKSEL0") (partNum 1) (symPinNum 10) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "11" (pinName "NRST") (partNum 1) (symPinNum 11) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "12" (pinName "RESV_NC_4") (partNum 1) (symPinNum 12) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "13" (pinName "RESV_NC_5") (partNum 1) (symPinNum 13) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "14" (pinName "H_INTN") (partNum 1) (symPinNum 14) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "15" (pinName "ENV_SCL ") (partNum 1) (symPinNum 15) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "16" (pinName "ENV_SDA") (partNum 1) (symPinNum 16) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "17" (pinName "SA0/H_MOSI ") (partNum 1) (symPinNum 17) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "18" (pinName "H_CSN") (partNum 1) (symPinNum 18) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "19" (pinName "H_SCL/SCK/RX") (partNum 1) (symPinNum 19) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "20" (pinName "H_SDA/H_MISO/TX ") (partNum 1) (symPinNum 20) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "21" (pinName "RESV_NC_6") (partNum 1) (symPinNum 21) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "22" (pinName "RESV_NC_7") (partNum 1) (symPinNum 22) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "23" (pinName "RESV_NC_8") (partNum 1) (symPinNum 23) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "24" (pinName "RESV_NC_9") (partNum 1) (symPinNum 24) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "25" (pinName "GND_2") (partNum 1) (symPinNum 25) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "26" (pinName "XOUT32/CLKSEL1") (partNum 1) (symPinNum 26) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "27" (pinName "XIN32") (partNum 1) (symPinNum 27) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "28" (pinName "VDDIO") (partNum 1) (symPinNum 28) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "BNO085"))
		(attachedPattern (patternNum 1) (patternName "BNO085")
			(numPads 28)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
				(padNum 5) (compPinRef "5")
				(padNum 6) (compPinRef "6")
				(padNum 7) (compPinRef "7")
				(padNum 8) (compPinRef "8")
				(padNum 9) (compPinRef "9")
				(padNum 10) (compPinRef "10")
				(padNum 11) (compPinRef "11")
				(padNum 12) (compPinRef "12")
				(padNum 13) (compPinRef "13")
				(padNum 14) (compPinRef "14")
				(padNum 15) (compPinRef "15")
				(padNum 16) (compPinRef "16")
				(padNum 17) (compPinRef "17")
				(padNum 18) (compPinRef "18")
				(padNum 19) (compPinRef "19")
				(padNum 20) (compPinRef "20")
				(padNum 21) (compPinRef "21")
				(padNum 22) (compPinRef "22")
				(padNum 23) (compPinRef "23")
				(padNum 24) (compPinRef "24")
				(padNum 25) (compPinRef "25")
				(padNum 26) (compPinRef "26")
				(padNum 27) (compPinRef "27")
				(padNum 28) (compPinRef "28")
			)
		)
		(attr "Mouser Part Number" "526-BNO085")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/CEVA/BNO085?qs=ulEaXIWI0c9BFVeZDQCmmQ%3D%3D")
		(attr "Manufacturer_Name" "CEVA")
		(attr "Manufacturer_Part_Number" "BNO085")
		(attr "Description" "Board Mount Motion & Position Sensors 9-axis IMU")
		(attr "<Hyperlink>" "https://www.ceva-dsp.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf")
		(attr "<Component Height>" "1.18")
		(attr "<STEP Filename>" "BNO085.stp")
		(attr "<STEP Offsets>" "X=0;Y=0;Z=0")
		(attr "<STEP Rotation>" "X=90;Y=0;Z=90")
	)

)