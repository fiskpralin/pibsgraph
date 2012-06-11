#! /bin/bash
#Script to run the simulations in Python from outside instead of from inside
#This ignores the problem with memory as python is closed between separate simulations.

heads="BC convAcc"
rowNumber=1
iterations=100
for treeFile in 210 102 103 403 105 304;do
	for head in $heads;do
		for nCranes in {1..2};do
			for bundler in {1..2};do
				for twigCrack in {1..2};do
					for simNumber in {1..100};do #here it is relevant to set the number of iterations for each machine and tree configuration
						#echo btDCTM $iterations $head $nCranes $bundler $twigCrack $simNumber $rowNumber $treeFile
						python main.py btDCTM $iterations $head $nCranes $bundler $twigCrack $simNumber $rowNumber $treeFile
						rowNumber=`echo $rowNumber+1 | bc`
					done
				done
			done
		done
	done
done

TXT="That was all the simulations. Now run from bash and seemingly without memory issues."
echo $TXT

