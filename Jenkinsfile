pipeline {
	agent any
	
	options {
		buildDiscarder(logRotator(numToKeepStr:'9'))
	}
	
	
	stages {
		stage ('Start') {
			steps {
				// send build started notifications
				slackSend (color: '#FFFF00', message: "GitHub Public RENEWLab Build STARTED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
				
				// send to email
				emailext (subject: "GitHub Public RENEWLab Build STARTED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]'",
					body: """<p>STARTED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]':</p>
						<p>Check console output at &QUOT;<a href='${env.BUILD_URL}'>${env.JOB_NAME} [${env.BUILD_NUMBER}]</a>&QUOT;</p>""",
					recipientProviders: [[$class: 'DevelopersRecipientProvider']])
			}
		}
		
		stage('Preparation') {
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
	
	
	post {
		success {
			slackSend (color: '#00FF00', message: "GitHub Public RENEWLab Build SUCCESSFUL: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
			
			emailext (subject: "GitHub Public RENEWLab Build SUCCESSFUL: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]'",
				body: """<p>SUCCESSFUL: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]':</p>
					<p>Check console output at &QUOT;<a href='${env.BUILD_URL}'>${env.JOB_NAME} [${env.BUILD_NUMBER}]</a>&QUOT;</p>""",
				recipientProviders: [[$class: 'DevelopersRecipientProvider']])
		}
		
		failure {
			slackSend (color: '#FF0000', message: "GitHub Public RENEWLab Build FAILED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]' (${env.BUILD_URL})")
			
			emailext (subject: "FAILED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]'",
				  body: """<p>FAILED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]':</p>
				  <p>Check console output at &QUOT;<a href='${env.BUILD_URL}'>${env.JOB_NAME} [${env.BUILD_NUMBER}]</a>&QUOT;</p>""",
				  recipientProviders: [[$class: 'DevelopersRecipientProvider']])
		}
	}
	
	
}

