def findLogFile() {
	jobName = "${env.JOB_NAME}"
	tokens = jobName.split('/')
	jobDir = tokens[0]
	filePath = "${env.JENKINS_HOME}/jobs/${jobDir}/branches/${env.JOB_BASE_NAME}/builds/${env.BUILD_NUMBER}/log"
	
	return filePath
}


pipeline {
	agent any
	
	options {
		skipDefaultCheckout true
		buildDiscarder(logRotator(numToKeepStr:'9'))
	}
	
	
	stages {
		stage ("Start") {
			steps {
				echo "CI started ..."
				slackSend (color: '#FFFF00', message: "Build STARTED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
			}
		}
		
		stage ("Fix Jenkins Changelog Bug") {
			when { expression { return !currentBuild.previousBuild } }
			steps {
				echo "CI preparation: Add Changelog to fix Jenkins' First-time-build bug ..."
				checkout([
					$class: 'GitSCM',
					branches: scm.branches,
					userRemoteConfigs: scm.userRemoteConfigs,
					browser: scm.browser,
					// build the changesets from the compareTarget branch
					extensions: [[$class: 'ChangelogToBranch', options: [compareRemote: 'origin', compareTarget: 'master']]]
				])
			}
		}
		
		// perform the normal configured checkout to ensure all configured extensions runs and to generate the changeset for later builds
		stage ("Checkout Source") {
			steps {
				echo "CI checking out from the source ..."
				checkout scm
			}
		}
		
		stage("Build mufft") {
			steps {
				echo "CI building mufft ..."
				dir ("CC/Sounder/mufft") {
					sh "git submodule update --init"
					sh "cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ./ && make -j"
				}
			}
		}
		
		stage("Build Sounder") {
			steps {
				echo "CI building Sounder ..."
				dir("CC/Sounder") {
					sh "cmake ./ && make -j"
				}				
			}
		}
		
		stage("Tests") {
			steps {
				dir("CC/Sounder/tests/comms-func") {
					echo "CI building tests ..."
					sh "cmake ./ && make -j"
					echo "CI Sounder testing ..."
					sh "./comm-testbench"
					script {
						// logFile = findLogFile()
						command = $/tail -60 ${findLogFile()} | grep -i 'test passed'/$
						pf_flag = sh(script: command, returnStdout: true)
						pf_flag = pf_flag.trim()
						if (pf_flag == "TEST PASSED") {
							echo "Passing due to " + pf_flag
							currentBuild.result = "SUCCESS"
						} else {
							echo "Failing due to " + pf_flag
							currentBuild.result = "FAILURE"
						}
					}
				}
			}
		}
	}
	
	
	post {
		success {
			echo "CI passed!"
			slackSend (color: '#00FF00', message: "Build SUCCESSFUL: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
		}
		
		failure {
			echo "CI failed!"
			slackSend (color: '#FF0000', message: "Build FAILED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
		}
	}
}

