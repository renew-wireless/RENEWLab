node('master') {
	
	checkout scm
	
	stage('Build') {
		stage("Get mufft dependencies and build"){
			dir ('CC/Sounder/mufft') {
				sh "git submodule update --init"
				sh "cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ./ && make -j"
			}
		}
		
		stage("Build Sounder"){
			dir('CC/Sounder') {
				sh "cmake ./ && make -j"
			}
		}

	}
	
}