pipeline {
	agent any
	
	options {
		buildDiscarder(logRotator(numToKeepStr:'9'))
	}
	
	
	stages {
		stage ("Start") {
			steps {
				echo "CI started ..."
				slackSend (color: '#FFFF00', message: "GitHub Public RENEWLab Build STARTED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
			}
		}
		
		stage("Preparation") {
			steps {
				echo "CI preparation ..."
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
						jobName = "${env.JOB_NAME}"
						tokens = jobName.split('/')
						jobDir = tokens[0]
						logFile = "${env.JENKINS_HOME}/jobs/${jobDir}/branches/${env.JOB_BASE_NAME}/builds/${env.BUILD_NUMBER}/log"
						echo logFile
						command_to_get_pf = $/tail -60 ${logFile} | grep -i 'test passed'/$
						pf_flag = sh(script: command_to_get_pf, returnStdout: true)
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
			slackSend (color: '#00FF00', message: "GitHub Public RENEWLab Build SUCCESSFUL: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
		}
		
		failure {
			echo "CI failed!"
			slackSend (color: '#FF0000', message: "GitHub Public RENEWLab Build FAILED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
		}
	}
	
	
}

