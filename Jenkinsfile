pipeline {
	agent any
	
	checkout scm
	
	stage("Preparation"){
		options {
			buildDiscarder(logRotator(numToKeepStr:'10'))
		}

	// 	sh 'sed -i -e "s/jenkins_ci_branch_name/$BRANCH_NAME/1" README.md'
	}
	
	stage("Build mufft"){
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
		
	// options {
	// 	buildDiscarder(logRotator(numToKeepStr: '9'))
	// }

}
