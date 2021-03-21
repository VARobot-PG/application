#!/usr/bin/env bash

set -x

echo "Testing for $TEST_PLATFORM"

CODE_COVERAGE_PACKAGE="com.unity.testtools.codecoverage"
PACKAGE_MANIFEST_PATH="Packages/manifest.json"

${UNITY_EXECUTABLE:-xvfb-run --auto-servernum --server-args='-screen 0 640x480x24' /opt/Unity/Editor/Unity} \
  -projectPath $(pwd) \
  -runTests \
  -testPlatform $TEST_PLATFORM \
  -testResults $(pwd)/$TEST_PLATFORM-results.xml \
  -logFile /dev/stdout \
  -batchmode \
  -enableCodeCoverage \
  -coverageResultsPath $(pwd)/$TEST_PLATFORM-coverage \
  -coverageOptions "generateAdditionalMetrics;generateHtmlReport;generateHtmlReportHistory;generateBadgeReport;assemblyFilters:+Assembly-CSharp" \
  -debugCodeOptimization

UNITY_EXIT_CODE=$?

if [ $UNITY_EXIT_CODE -eq 0 ]; then
  echo "Run succeeded, no failures occurred";
elif [ $UNITY_EXIT_CODE -eq 2 ]; then
  echo "Run succeeded, some tests failed";
elif [ $UNITY_EXIT_CODE -eq 3 ]; then
  echo "Run failure (other failure)";
else
  echo "Unexpected exit code $UNITY_EXIT_CODE";
fi

if grep $CODE_COVERAGE_PACKAGE $PACKAGE_MANIFEST_PATH; then
  cat $(pwd)/$TEST_PLATFORM-coverage/Report/Summary.xml | grep Linecoverage
  mv $(pwd)/$TEST_PLATFORM-coverage/$CI_PROJECT_NAME-opencov/*Mode/TestCoverageResults_*.xml $(pwd)/$TEST_PLATFORM-coverage/coverage.xml
  rm -r $(pwd)/$TEST_PLATFORM-coverage/$CI_PROJECT_NAME-opencov/
else
  { 
    echo -e "\033[33mCode Coverage package not found in $PACKAGE_MANIFEST_PATH. Please install the package \"Code Coverage\" through Unity's Package Manager to enable coverage reports.\033[0m" 
  } 2> /dev/null
fi

cat $(pwd)/$TEST_PLATFORM-results.xml | grep test-run | grep Passed
apt-get update -y
apt-get install -y xsltproc
xsltproc ./ci/report/nunit-to-junit.xsl $(pwd)/$TEST_PLATFORM-results.xml > $(pwd)/$TEST_PLATFORM-junit-results.xml
#!/bin/bash
xsltproc testResultsToFailedOnly.xsl playmode-junit-results.xml > failed.txt
myarray1=(`cat NotRequiredTests.txt`) 
noofelements1=${#myarray1[*]}
myarray2=(`cat failed.txt`) 
noofelements2=${#myarray2[*]}
#now traverse the array
i=0
a=1;
while [ $i -lt $noofelements2 ]
do
    j=0
    c=0
    while [ $j -lt $noofelements1 ]
    do
        echo " (Failed) Element $j is  A${myarray1[$j]}A"
        echo " (NotReq) Element $i is  A${myarray2[$i]}A"
        if [[ "$myarray1[$j]" = "$myarray2[$i]" ]];
        then
            c=1;
            echo "is not eq"
        else 
            echo "is eq"
        fi
        j=$(( $j + 1 ))
    done
    if [ ${c} -eq  0 ]
    then
        a=0;
    fi
    i=$(( $i + 1 ))
done
if [ ${a} -eq  0 ]
then
  echo "required tests failed"
else 
  echo "no required tests failed"
  UNITY_EXIT_CODE=0;
fi

exit $UNITY_EXIT_CODE
