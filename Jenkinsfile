node('master') {
	
	checkout scm
	
	stage('Build') {
		stage("Preparation"){
			// sh "sed -i -e 's/jenkins_ci_branch_name/$BRANCH_NAME/g' README.md"
		}
		
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
