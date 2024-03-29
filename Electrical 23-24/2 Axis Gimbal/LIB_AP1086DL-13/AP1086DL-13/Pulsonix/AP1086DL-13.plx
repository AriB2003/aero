PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//2407493/1276551/2.50/3/4/Integrated Circuit

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r200_120"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 1.2) (shapeHeight 2))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(padStyleDef "s560"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 5.6) (shapeHeight 5.6))
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
	(patternDef "TO252-3L_1" (originalName "TO252-3L_1")
		(multiLayer
			(pad (padNum 1) (padStyleRef r200_120) (pt -2.25, -8.7) (rotation 0))
			(pad (padNum 2) (padStyleRef r200_120) (pt 2.25, -8.7) (rotation 0))
			(pad (padNum 3) (padStyleRef s560) (pt 0, -1.5) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt -0.341, -7.069) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef 28)
			(line (pt -3.4 0) (pt 3.4 0) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 3.4 0) (pt 3.4 -5.8) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 3.4 -5.8) (pt -3.4 -5.8) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -3.4 -5.8) (pt -3.4 0) (width 0.025))
		)
		(layerContents (layerNumRef 18)
			(line (pt -3.4 0) (pt -3.4 -5.8) (width 0.254))
		)
		(layerContents (layerNumRef 18)
			(line (pt -3.4 -5.8) (pt 3.4 -5.8) (width 0.254))
		)
		(layerContents (layerNumRef 18)
			(line (pt 3.4 -5.8) (pt 3.4 0) (width 0.254))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -2.335, -10.353) (radius 0.07273) (startAngle 0.0) (sweepAngle 0.0) (width 0.254))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -2.335, -10.353) (radius 0.07273) (startAngle 180.0) (sweepAngle 180.0) (width 0.254))
		)
	)
	(symbolDef "AP1086DL-13" (originalName "AP1086DL-13")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 1400 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 100 mils) (pt 1200 mils 100 mils) (width 6 mils))
		(line (pt 1200 mils 100 mils) (pt 1200 mils -200 mils) (width 6 mils))
		(line (pt 1200 mils -200 mils) (pt 200 mils -200 mils) (width 6 mils))
		(line (pt 200 mils -200 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 1250 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 1250 mils 200 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "AP1086DL-13" (originalName "AP1086DL-13") (compHeader (numPins 3) (numParts 1) (refDesPrefix IC)
		)
		(compPin "1" (pinName "ADJ (GND)") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "VIN") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "3" (pinName "VOUT") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "AP1086DL-13"))
		(attachedPattern (patternNum 1) (patternName "TO252-3L_1")
			(numPads 3)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
			)
		)
		(attr "Mouser Part Number" "621-AP1086DL-13")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Diodes-Incorporated/AP1086DL-13?qs=EJYhozv%252BYpwmQ8i36Rjd6Q%3D%3D")
		(attr "Manufacturer_Name" "Diodes Incorporated")
		(attr "Manufacturer_Part_Number" "AP1086DL-13")
		(attr "Description" "LDO Voltage Regulators LDO BI 1.5A 1.4V 12V 1.25V ADJ")
		(attr "<Hyperlink>" "https://datasheet.datasheetarchive.com/originals/distributors/Datasheets-DIO/DSADIO2000107.pdf")
	)

)
