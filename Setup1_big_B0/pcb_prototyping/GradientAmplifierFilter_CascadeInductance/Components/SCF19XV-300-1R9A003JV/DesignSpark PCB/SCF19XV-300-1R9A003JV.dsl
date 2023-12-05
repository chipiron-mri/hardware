SamacSys ECAD Model
17076275/1223673/2.50/4/2/Filter

DESIGNSPARK_INTERMEDIATE_ASCII

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "c315_h210"
		(holeDiam 2.1)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 3.150) (shapeHeight 3.150))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 3.150) (shapeHeight 3.150))
	)
	(padStyleDef "s315_h210"
		(holeDiam 2.1)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 3.150) (shapeHeight 3.150))
		(padShape (layerNumRef 16) (padShapeType Rect)  (shapeWidth 3.150) (shapeHeight 3.150))
	)
	(textStyleDef "Default"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 50 mils)
			(strokeWidth 5 mils)
		)
	)
	(patternDef "SCF19XV3001R9A003JV" (originalName "SCF19XV3001R9A003JV")
		(multiLayer
			(pad (padNum 1) (padStyleRef s315_h210) (pt 0.000, 0.000) (rotation 90))
			(pad (padNum 2) (padStyleRef c315_h210) (pt 0.000, -12.000) (rotation 90))
			(pad (padNum 3) (padStyleRef c315_h210) (pt 17.000, -12.000) (rotation 90))
			(pad (padNum 4) (padStyleRef c315_h210) (pt 17.000, 0.000) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 8.500, -6.000) (textStyleRef "Default") (isVisible True))
		)
		(layerContents (layerNumRef 28)
			(line (pt -5.5 -15.2) (pt 22.5 -15.2) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 22.5 -15.2) (pt 22.5 3.2) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 22.5 3.2) (pt -5.5 3.2) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -5.5 3.2) (pt -5.5 -15.2) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt -6.5 4.2) (pt 23.5 4.2) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt 23.5 4.2) (pt 23.5 -16.2) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt 23.5 -16.2) (pt -6.5 -16.2) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt -6.5 -16.2) (pt -6.5 4.2) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -5.5 3.2) (pt 22.5 3.2) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 22.5 3.2) (pt 22.5 -15.2) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 22.5 -15.2) (pt -5.5 -15.2) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -5.5 -15.2) (pt -5.5 3.2) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 0 3.8) (pt 0 3.8) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(arc (pt 0, 3.75) (radius 0.05) (startAngle 90.0) (sweepAngle 180.0) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt 0 3.7) (pt 0 3.7) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(arc (pt 0, 3.75) (radius 0.05) (startAngle 270) (sweepAngle 180.0) (width 0.1))
		)
	)
	(symbolDef "SCF19XV-300-1R9A003JV" (originalName "SCF19XV-300-1R9A003JV")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 3) (pt 1100 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 4) (pt 1100 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(line (pt 200 mils 100 mils) (pt 900 mils 100 mils) (width 6 mils))
		(line (pt 900 mils 100 mils) (pt 900 mils -200 mils) (width 6 mils))
		(line (pt 900 mils -200 mils) (pt 200 mils -200 mils) (width 6 mils))
		(line (pt 200 mils -200 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 950 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Default"))

	)
	(compDef "SCF19XV-300-1R9A003JV" (originalName "SCF19XV-300-1R9A003JV") (compHeader (numPins 4) (numParts 1) (refDesPrefix FL)
		)
		(compPin "1" (pinName "N1_1") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "2" (pinName "N1_2") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "3" (pinName "N2_1") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "4" (pinName "N2_2") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "SCF19XV-300-1R9A003JV"))
		(attachedPattern (patternNum 1) (patternName "SCF19XV3001R9A003JV")
			(numPads 4)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
			)
		)
		(attr "Mouser Part Number" "80-SCF19XV3001R903JV")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/KEMET/SCF19XV-300-1R9A003JV?qs=MyNHzdoqoQJ8dcdKI%2FcPyQ%3D%3D")
		(attr "Manufacturer_Name" "KEMET")
		(attr "Manufacturer_Part_Number" "SCF19XV-300-1R9A003JV")
		(attr "Description" "KEMET, SCF-XV, AC Line Filters, Common Mode, 0.11 mH ")
		(attr "Datasheet Link" "https://connect.kemet.com:7667/gateway/IntelliData-ComponentDocumentation/1.0/download/datasheet/SCF19XV-300-1R9A003JV.pdf")
		(attr "Height" "27.5 mm")
	)

)