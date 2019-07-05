#!/usr/bin/python
#
#    Run the trigger generator on every node found, or every specified node
#    Sort nodes as "arrays", where arrays contain multiple devices that trigger each other.
#    Print all of the arrays separately, in order of who triggers who (their topology).
#    If hubs exist, then trigger the hub and find the sub-arrays.
#
#    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#    INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#    PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#    FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#    OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#    DEALINGS IN THE SOFTWARE.
#
#    (c) 2018 info@skylarkwireless.com

import sys
import time
import threading
import SoapySDR
from SoapySDR import * #SOAPY_SDR_ constants

def testTriggers(sdrs, which):
	timeLastTriggered0 = [sdr.getHardwareTime("TRIGGER") for sdr in sdrs]

	time.sleep(0.1)
	sdrs[which].writeSetting("TRIGGER_GEN", "")
	time.sleep(.05)
	timeLastTriggered1 = [sdr.getHardwareTime("TRIGGER") for sdr in sdrs]
	#print(which,timeLastTriggered0)
        #print(which,timeLastTriggered1)
	indexes = list()
	for i, (x0, x1) in enumerate(zip(timeLastTriggered0, timeLastTriggered1)):
		if x0 != x1: indexes.append(i)

	return indexes

def setupSDR(sdr):
	try:
		sdr.setMasterClockRate(640e6)
		sdr.setSampleRate(SOAPY_SDR_RX, 0, 10e6)
		sdr.setSampleRate(SOAPY_SDR_TX, 0, 10e6)
	except: print("failed to init %s" % sdr.serial)

if __name__ == '__main__':

	if len(sys.argv) == 1:
		print('Automatically finding all SDRs on network...')
		handles = SoapySDR.Device.enumerate({"remote:timeout":"250000"})
		serials = [s['serial'] for s in handles]

	else:
		serials = sys.argv[1:]
		handles = [dict(driver='iris',serial=s) for s in serials]
	sdrs = SoapySDR.Device(handles)
	for s in sdrs: s.serial = s.getHardwareInfo()['serial']

	#print("\n".join([str(s.getHardwareInfo()) for s in sdrs]))

	irises = [s for s in sdrs if 'Iris' in s.getHardwareInfo()['revision'] ]
	hubs = [s for s in sdrs if 'FAROS-HUB' in s.getHardwareInfo()['revision'] ]

	threads = [threading.Thread(target=setupSDR, args=[sdr]) for sdr in irises] 
	for t in threads: t.start()
	for t in threads: t.join()

	trig_list = [testTriggers(irises, i) for i in range(len(irises))]
	#print([s.serial for s in irises])
	#print(trig_list)
	head_of_chain = [True]*len(irises)

	#look at the devices each sdr triggered.
	#any devices triggered downstream are not the head of chain
	for i in range(len(irises)):
		for j in trig_list[i]:
			if j != i:
				head_of_chain[j] = False

	#print(head_of_chain)
	print('\n%d Arrays discovered:\n' % sum(head_of_chain))

	#For each array we found correctly order the chain and print it
	serials_of_chain = [[] for i in range(len(head_of_chain))]
	for i in range(len(head_of_chain)):
		if head_of_chain[i]:
			#look at how many other sdrs each device triggered -- the more it triggered the nearer it is to the head.
			ordered = sorted([(len(trig_list[j]),j) for j in trig_list[i]], reverse=True) #ordered contains a list if [num_triggered, index_of_iris] in descending order of num_triggered
			#print(ordered)
			serials_of_chain[i] = [irises[s[1]].serial for s in ordered] #s[1] is the index of the iris in irises
			print(" ".join(serials_of_chain[i]))

	if len(hubs) == 0:
		sys.exit(0)

	print("\nHubs: \n")

	for h in hubs:
		hub_trig_list = testTriggers(irises + [h], len(irises)) #trigger the hub and see which irises respond, add to end so indexing isn't changed.
		#print(hub_trig_list)
		print(h.serial + ":")
		for i in hub_trig_list:
			if head_of_chain[i]:
				print("\t" + " ".join(serials_of_chain[i]))
		print("")

	#TODO: Add sanity checks to make sure every board appears once, and no board appears twice.
