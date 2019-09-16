/*
 * Amazon FreeRTOS V1.1.4
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */


/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Test framework includes. */
#include "unity_fixture.h"

#define ESP_TEST_DEFAULT_PRIORITY (tskIDLE_PRIORITY + 1)
#define ESP_TEST_HIGH_PRIORITY (tskIDLE_PRIORITY + 3)
#define ESP_TASK_DEFAULT_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)

#define ESP_TEST_PINNED_TASK_NAME "ESP_Pin"
#define TASK_CREATION_TIMEOUT pdMS_TO_TICKS(1000)
/**
 * @brief Test group for ESP Task creation tests
 */
TEST_GROUP( ESP_SMP_Task_Creation );

/**
 * @brief Test group runner for ESP SMP task creation
 */
TEST_GROUP_RUNNER( ESP_SMP_Task_Creation )
{
    RUN_TEST_CASE( ESP_SMP_Task_Creation, IsMultiCore );
    RUN_TEST_CASE( ESP_SMP_Task_Creation, CreatePinnedTaskOnCore0 );
    RUN_TEST_CASE( ESP_SMP_Task_Creation, CreatePinnedTaskOnCore1 );
}

TEST_SETUP( ESP_SMP_Task_Creation) {}
TEST_TEAR_DOWN( ESP_SMP_Task_Creation) {}

/**
 * @brief The response structure from report core
 */
typedef struct {
    SemaphoreHandle_t done; /*< The reportCore task sets this semaphore on completion */
	BaseType_t			xCoreID; /*< The CPU ID on which report Core ran */
} coreReport_t;

/**
 * @brief Record the core on which this task runs, then suspend
 */
void reportCore( void* pvReport ) {
    coreReport_t *  pxReport = (coreReport_t *) pvReport;
    pxReport->xCoreID =  xPortGetCoreID();
    if ( pxReport -> done ) {
        if ( !xSemaphoreGive( pxReport -> done )) {
            configPRINTF(("Error when returning semaphore."));
        }
    }
    vTaskSuspend(NULL);
}

/**
 * @brief Test whether pinned tasks run on the core to which they were assigned
 */
void TestCreatePinnedTask( BaseType_t xCoreID, char* taskName ) {
    #define CORE_NOT_SET INT_MAX
    TaskHandle_t xTaskHandle = NULL;
    BaseType_t taskCreated = pdFALSE, taskCompleted = pdFALSE;
    coreReport_t _coreReport = {
        .xCoreID = CORE_NOT_SET,
        .done = xSemaphoreCreateBinary()
    };

    coreReport_t * coreReport = &_coreReport;

    if (TEST_PROTECT()) {

        TEST_ASSERT_NOT_NULL_MESSAGE( coreReport -> done, "creating semaphore" );

        taskCreated =  xTaskCreatePinnedToCore( reportCore,
                                        taskName,
                                        ESP_TASK_DEFAULT_STACK_SIZE,
                                        coreReport,
                                        ESP_TEST_DEFAULT_PRIORITY,
                                        &xTaskHandle,
                                        xCoreID);
        TEST_ASSERT_TRUE_MESSAGE( taskCreated, "creating task" );

        taskCompleted = xSemaphoreTake(  coreReport -> done , TASK_CREATION_TIMEOUT);

        TEST_ASSERT_TRUE_MESSAGE( taskCompleted, "Reporting task timed out");
        TEST_ASSERT_EQUAL_MESSAGE( coreReport -> xCoreID, xCoreID, "Core ID for pinned task");
    }

    /* Cleanup */
    if (xTaskHandle != NULL) { 
        vTaskDelete( xTaskHandle );
    }

    if ( coreReport -> done ) {
        vSemaphoreDelete( coreReport -> done );
    }

}

/**
 * @brief Test that the multicore configuration flag is enabled in this build
 */
TEST( ESP_SMP_Task_Creation, IsMultiCore ) {
    TEST_ASSERT_GREATER_THAN( 1,  portNUM_PROCESSORS );
}

TEST( ESP_SMP_Task_Creation, CreatePinnedTaskOnCore0 )
{
    TestCreatePinnedTask(0, ESP_TEST_PINNED_TASK_NAME "_0" );
}

TEST( ESP_SMP_Task_Creation, CreatePinnedTaskOnCore1 )
{
    TestCreatePinnedTask(1, ESP_TEST_PINNED_TASK_NAME "_1" );
}

