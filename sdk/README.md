# Incessus SDK U9/U10/U11/U12 - 2026-10-05

Contrato para as DLLs Incessus U9/U10/U11/U12 em teste (runtime Skyrim 1.6.640.0). A SDK � somente leitura. Inclua `include/IncessusAPI.h`; `Incessus::API` � um alias do header p�blico real do produtor, `AnimalLocomotionRevolutionAPI.h`. N�o mantenha uma segunda defini��o das structs no consumidor.

## Conex�o e consulta

Resolva `ALR_RequestInterface` em `Incessus.dll` com GetModuleHandle/GetProcAddress. O nome hist�rico do export permanece por compatibilidade. Solicite `Incessus::API::kInterfaceVersion1`, valide com `Incessus::SDK::Compatible` e consulte com `Incessus::SDK::Query(api, actorFormID, snapshot)`.

Layout Windows x64 atual: LocomotionSnapshotV1 **408 bytes**, InterfaceV1 **40 bytes**. O Forma R5.7 tinha uma c�pia de 192 bytes: QueryLocomotion recusa esse buffer antes de escrever, fazendo o consumidor perder os snapshots. R5.8 usa o header completo e valida tamanho, vers�o, FormID, serial e flag de suporte na resposta. Uma consulta inv�lida devolve um snapshot limpo.

V1 historicamente recebeu uma extens�o sem mudar seu n�mero de vers�o. Esta entrega mant�m o ABI da DLL atualmente em teste e fixa seu layout; a pr�xima altera��o incompat�vel deve criar outra vers�o de interface, n�o modificar novamente V1. Consumidores compilados com o header antigo precisam ser recompilados.

## Dados efetivamente dispon�veis

- Snapshots publicados ap�s a atualiza��o do ator, quando existe consumidor conectado. Uma consulta inicial antes da primeira publica��o pode retornar false.
- Posi��o/orienta��o, inten��o de marcha, metas de velocidade do perfil, par�metros de steering, flags de a��o/turn e testemunhas dos carriers existentes.
- �ngulos em radianos, posi��es em unidades Skyrim. Inten��o de marcha n�o equivale a movimento f�sico comprovado; bAnimationDriven n�o prova que h� deslocamento.
- Os flags com nomes hist�ricos DB/External t�m a sem�ntica dos witnesses herdados. Eles n�o significam que uma curva externa esteja sendo injetada nesta DLL.

Na U10, `kGraphLocomotionActive` usa o parent locomotor realmente observado quando h� contrato confirmado.
Se o perfil usava o mesmo `bAnimationDriven` para indicar ownership e locomo��o, a aus�ncia de parent
observado exige um carrier locomotor certificado. Ownership permanente, sozinho, n�o marca idle/turn como
caminhada. Perfis de refer�ncia com witness pr�prio mant�m esse witness. O ABI V1 de 408 bytes n�o mudou.

Os campos finais `nativeMotionValid`, `nativeAuthoredYawValid`, `nativeFacingYawValid`, `nativeLocomotionOwned`, deltas/speed e nomes de node/asset **n�o s�o preenchidos pelo produtor U9/U10/U11/U12 atual**. Permanecem zero; o consumidor deve respeitar os campos de validade. N�o anunciar esses campos como telemetria nativa dispon�vel.

U11 mant�m a corre��o de locomo��o da U10 e o mesmo ABI. A nova captura de comandos nativos e os handlers de transi��o s�o internos ao Incessus. Forma R5.8 continua sendo o consumidor compat�vel; o teste visual conjunto ainda � necess�rio.

U12 corrige a entrega de inten��o �s m�quinas normais e retira o veto de eventos nativos. N�o altera o header p�blico, o ABI ou o algoritmo do Forma. No teste U11 a DLL Forma efetiva ainda era anterior � R5.8; o resultado conjunto R5.8 permanece pendente.

QueryClipMetrics/SampleCumulative s�o exports legados. Com curvas externas desativadas podem retornar aus�ncia de dados. O consumidor n�o deve exigir sucesso dessas consultas para se conectar. O Forma mant�m seu fallback de ritmo do perfil/graph; isso n�o reintroduz bootstrap de curvas nem concede autoridade sobre root/c�psula.

## Entrega e verifica��o

Forma Motus R5.8 usa esta SDK e mant�m o escopo procedural n�o humano j� existente. Teste de ABI reproduz recusa do buffer antigo, protege sentinelas de mem�ria e verifica consulta atual e respostas inv�lidas. Build e teste local n�o provam resultado visual no Skyrim.

A SDK n�o instala hooks nem modifica a DLL Incessus em teste. Headers de refer�ncia do Havok/CommonLib usados internamente pelo mod s�o uma interface distinta desta API p�blica.

