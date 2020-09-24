pipeline {
	agent any
	
	options {
		buildDiscarder(logRotator(numToKeepStr:'10'))
	}
	
	stages {
		stage('Preparaion') {
			steps {
				echo 'Preparation ...'
			}
		}
			
		stage('Build mufft') {
			steps {
				echo 'Build mufft ...'
				dir ('CC/Sounder/mufft') {
					sh "git submodule update --init"
					sh "cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ./ && make -j"
				}
			}
		}
		
		stage('Build Sounder') {
			steps {
				echo 'Build Sounder ...'
				dir('CC/Sounder') {
					sh "cmake ./ && make -j"
				}				
			}
		}
	}
	
}
