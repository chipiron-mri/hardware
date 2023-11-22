PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//644690/1223673/2.50/6/4/Connector

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "c396_h264"
		(holeDiam 2.64)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 3.960) (shapeHeight 3.960))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 3.960) (shapeHeight 3.960))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "7807" (originalName "7807")
		(multiLayer
			(pad (padNum 1) (padStyleRef c396_h264) (pt 0.000, 0.000) (rotation 90))
			(pad (padNum 2) (padStyleRef c396_h264) (pt 0.000, 10.800) (rotation 90))
			(pad (padNum 3) (padStyleRef c396_h264) (pt 5.210, 0.000) (rotation 90))
			(pad (padNum 4) (padStyleRef c396_h264) (pt 5.210, 10.800) (rotation 90))
			(pad (padNum 5) (padStyleRef c396_h264) (pt 10.420, 0.000) (rotation 90))
			(pad (padNum 6) (padStyleRef c396_h264) (pt 10.420, 10.800) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 5.210, 5.400) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef 28)
			(line (pt -0.76 11.37) (pt 11.18 11.37) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 11.18 11.37) (pt 11.18 -0.57) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 11.18 -0.57) (pt -0.76 -0.57) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -0.76 -0.57) (pt -0.76 11.37) (width 0.025))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.98 13.78) (pt 13.4 13.78) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 13.4 13.78) (pt 13.4 -2.98) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 13.4 -2.98) (pt -2.98 -2.98) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.98 -2.98) (pt -2.98 13.78) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -0.76 8.4) (pt -0.79 2.4) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt 11.18 8.4) (pt 11.21 2.4) (width 0.1))
		)
	)
	(symbolDef "7807" (originalName "7807")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 800 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 570 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 4) (pt 800 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 570 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 5) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 6) (pt 800 mils -200 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 570 mils -225 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 100 mils) (pt 600 mils 100 mils) (width 6 mils))
		(line (pt 600 mils 100 mils) (pt 600 mils -300 mils) (width 6 mils))
		(line (pt 600 mils -300 mils) (pt 200 mils -300 mils) (width 6 mils))
		(line (pt 200 mils -300 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 650 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 650 mils 200 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "7807" (originalName "7807") (compHeader (numPins 6) (numParts 1) (refDesPrefix J)
		)
		(compPin "1" (pinName "1") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "2") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "3" (pinName "3") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "4" (pinName "4") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "5" (pinName "5") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "6" (pinName "6") (partNum 1) (symPinNum 6) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "7807"))
		(attachedPattern (patternNum 1) (patternName "7807")
			(numPads 6)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
				(padNum 5) (compPinRef "5")
				(padNum 6) (compPinRef "6")
			)
		)
		(attr "Mouser Part Number" "534-7807")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Keystone-Electronics/7807?qs=%2F%252BPMR94VuMaEJokheHBVWg%3D%3D")
		(attr "Manufacturer_Name" "Keystone Electronics")
		(attr "Manufacturer_Part_Number" "7807")
		(attr "Description" "KEYSTONE - 7807 - TERMINAL, PCB SCREW, 2.57MM")
		(attr "<Hyperlink>" "http://www.keyelco.com/product-pdf.cfm?p=1119")
		(attr "<Component Height>" "11.76")
		(attr "<STEP Filename>" "7807.stp")
		(attr "<STEP Offsets>" "X=0;Y=0;Z=0")
		(attr "<STEP Rotation>" "X=0;Y=0;Z=0")
	)

)