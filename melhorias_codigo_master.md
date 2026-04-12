# Proposta de Melhorias no Código (TMS_MASTER_AMP_226)

A arquitetura do Master possui uma complexidade maior, visto que ele orquestra todas as informações recebidas dos escravos. Abaixo estão listadas melhorias essenciais de estrutura e correções de bugs lógicos que encontrei no código.

## 1. Bug Lógico na Injeção de Falhas (`injectFault`)

Atualmente, na função `xSendCANFunction` você faz isso:
`injectFault(findMaxVal(slave1TempBuffer));`

E no arquivo `errors.c` a função está assim:
```c
void injectFault(int temp){
	if(simulateHighTemp){
		temp = 100; // <--- O valor muda aqui dentro, mas NÃO afeta nada lá fora
	}
    // ...
}
```
**O Problema**: A linguagem C funciona com passagem por **valor**. Alterar `temp` dentro da função não vai forçar o alarme de temperatura alta (`maxTemperatureThreshold`) logo abaixo no seu `main.c`. 

**A Solução**: Você precisa que a função retorne o novo valor ou use passagem por referência (ponteiro). Uma forma mais rápida (retorno):

*Em errors.c:*
```c
int injectFault(int temp){
	if(simulateHighTemp){
		return 100; // Retorna temperatura crítica simulada
	}
	else if(simulateCommLoss){
		slave1LastMessageTick = 0;
	}
    return temp; // Retorna a originial se não haver teste
}
```

*Em main.c (na Thread do CAN):*
```c
int maxSlave1 = injectFault(findMaxVal(slave1TempBuffer));

// Agora você usa o valor possivelmente injetado no if de segurança:
if(maxSlave1 > maxTemperatureThreshold || ... )
{
    tmsErrorCode = overTemperatureFault;
    Error_Handler();
}
```

---

## 2. Race Condition Recebendo Via Interrupção CAN

O seu `HAL_FDCAN_RxFifo0Callback` está rodando no ambiente de **Interrupção de Hardware** (ISR). Dentro desse Callback, você chama a função `receiveCANFromSlaves()` que atualiza os arrays `slaveTempBuffers`.

Ao mesmo tempo, a sua *Thread* `xSendCANFunction` no `main.c` fica constantemente lendo esses mesmos arrays chamando `findMaxVal()`. 
**Problema:** A *Thread* pode estar no meio de um laço lendo o array quando o processador interrompe tudo para atualizar o mesmo array (novo frame CAN chegando), quebrando os dados.

**Solução Ideal (Padrão RTOS):**
Em vez de atualizar as grandes matrizes logo dentro do Callback, crie uma **Fila (osMessageQueue)** para passar apenas a mensagem CAN crua para uma *Thread* separada dedicada ao recebimento, ou proteja a leitura na Thread usando seções críticas (`taskENTER_CRITICAL()`):

*Como proteger a leitura no main (Solução Rápida):*
```c
void xSendCANFunction(void *argument)
{
    for(;;)
    {
        // 1. Desabilita as interrupções momentaneamente para garantir não sermos atropelados
        taskENTER_CRITICAL(); 
        
        // 2. Faz as leituras complexas dos arrays com segurança máxima
        int maxS1 = findMaxVal(slave1TempBuffer);
        int maxS2 = findMaxVal(slave2TempBuffer);
        int maxS3 = findMaxVal(slave3TempBuffer);
        int maxS4 = findMaxVal(slave4TempBuffer);
        
        // 3. Devolvemos a CPU novamente!
        taskEXIT_CRITICAL();

        // 4. Injeta falhas se precisar no S1 local e testa
        maxS1 = injectFault(maxS1);

        sendMasterInfoToCAN(maxS1, maxS2, maxS3, maxS4, tmsErrorCode);
        
        if(maxS1 > maxTemperatureThreshold || maxS2 > maxTemperatureThreshold || maxS3 > maxTemperatureThreshold || maxS4 > maxTemperatureThreshold)
        {
            tmsErrorCode = overTemperatureFault;
            Error_Handler();
        }

        osDelay(100);
    }
}
```

---

## 3. Sincronização Correta da Tarefa (ADC -> FreeRTOS)

Assim como no código do *Slave*, a thread de leitura analógica do simulador Master também estava usando polling infinito com `osDelay(1)` ao invés de responder diretamente aos eventos do ADC.

```c
// Dentro de main.c
void xReadTempFunction(void *argument)
{
#ifdef simulateSlave
    static bool filtersInitialized = false;
#endif

    // Inicialize e pegue o ID da tarefa corrente!
    // xReadTempHandle = xTaskGetCurrentTaskHandle();

    for(;;)
    {
        // Trava a Thread e aguarda o ADC (Tirar o comentário daqui)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

#ifdef simulateSlave
        if(!filtersInitialized)
        {
            initTemperatureFilters(rawAdcBuffer);
            filtersInitialized = true;
        }

        for(int i = 0; i < numberOfThermistors; i++){
            // Lógicas do filtro e check
            // ...
        }
#endif
        // Remover o osDelay(1); do fim, pois o NotifyTake trava a tarefa no loop inicial!
    }
}
```

---

## 4. Organização do arquivo Main

A mesma recomendação do Slave vale para cá. Mover `xSendCANFunction`, `xCheckCommsFuncion` e `xReadTempFunction` para o respectivo **`app_freertos.c`** deixa o Master mais claro e legível, isolando o código da inicialização do processador em relação à lógica funcional de multitarefa. Não esqueça do `extern` nas variáveis acessadas de outros arquivos, como `FDCAN2TxData` etc.
